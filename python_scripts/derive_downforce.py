"""
Derive per-track (c_l, c_d) aero coefficients from a track's own real FastF1 telemetry,
instead of guessing which of three fixed tiers (main.rs's HIGH/MED/LOW_DOWNFORCE) a circuit
belongs to.

Why: validating the racing-line solver's lap times against real 2018 pole times (with the
3-tier system) showed a systematic +16.3% mean gap across all 15 working tracks, worse the
more corner-time-dominated the track (Silverstone/COTA/Hockenheim: +21-22%; Baku/Monza,
straight-line-dominated: +10%). Raising mu_lat toward the top of its published range closed
only part of it uniformly -- the tiers themselves were the bigger problem.

Two independent physical measurements per track, both from the SAME lap already exported by
export_track.py (data/<slug>_track_geometry.csv's X/Y and data/<slug>_reference_lap.csv's
Speed share the same Distance axis -- no cross-axis interpolation needed here, unlike the
OSM boundary data):

1. c_d from top speed: at the real observed top speed, power and drag balance
   (P = c_d_drs * m * v^3, in this codebase's per-unit-mass convention), assuming DRS is open
   there (the entire reason DRS exists is to raise top speed). v_max taken as the 99.5th
   percentile of smoothed speed, not the raw max, to avoid one noisy telemetry sample
   swinging a cubic relationship.

2. c_l from real apex lateral acceleration: the friction ellipse says a_lat = mu_lat*(G +
   c_l*v^2) at a point of pure cornering (a_lon ~ 0). Real corners rarely hold EXACTLY zero
   longitudinal acceleration for one instant, so instead of picking single local-minimum
   points (an earlier attempt at this: 0-9 samples per track, several tracks got zero),
   every point where |dv/ds| falls in the bottom 25th percentile for that lap counts as
   "quasi-steady-state cornering" -- this captures whole apex plateaus, not one point,
   giving dramatically more samples (2-34 per track here). Filtered to real corners only
   (curvature >= MIN_CURVATURE) fast enough that aero downforce is a meaningful fraction of
   total grip (>= MIN_APEX_SPEED_KMH) -- a slow hairpin is mechanical-grip-dominated and
   would bias c_l toward noise. Takes the per-track median of the implied c_l across all
   qualifying points.

X/Y and Speed are lightly smoothed (Savitzky-Golay, ~9-sample window) before differentiating
for curvature and dv/ds -- raw telemetry noise otherwise creates spurious "corners" and
unstable derivatives.

Validated end-to-end (not just that the per-corner numbers look plausible): plugging the
derived values into the racing-line solver dropped the mean gap to real 2018 pole times from
+16.3% to +0.9% across all 15 tracks (stdev roughly unchanged, ~4%, so this is closing the
gap broadly, not just moving it around) -- both c_l and c_d came from real telemetry signals
independent of lap time itself, so this isn't circular curve-fitting to the answer.

Known limitation: per-corner c_l implied values have wide spread within a single track (e.g.
Red Bull Ring's interquartile range spanned 0.005-0.021, over 4x) even after smoothing and
the quasi-steady-state filter -- taking the median is a reasonably robust point estimate, but
a single constant c_l can't represent real aero load variation across corner types (a wing's
downforce/drag characteristics aren't perfectly speed-invariant in reality). Power circuits
with few genuine high-speed corners (Monza, Montreal) get very few qualifying samples (2-3)
-- likely reflects genuine track character rather than a detection failure, but worth a
second look if a future track's derived value looks implausible.

Usage:
  python derive_downforce.py --track monaco

Requires data/<slug>_track_geometry.csv and data/<slug>_reference_lap.csv (export_track.py).
Outputs data/<slug>_aero_params.csv (c_l, c_d), read by src/main.rs (track::load_aero_params)
in place of the 3-tier fallback when present.
"""

import argparse

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

MU_LAT = 1.6
G = 9.81
P_ENGINE = 1015.0
DRS_DRAG_REDUCTION = 0.88  # matches main.rs's DRS_DRAG_REDUCTION

MIN_APEX_SPEED_KMH = 150.0
MIN_CURVATURE = 0.008
DVDS_PERCENTILE = 25  # bottom 25% of |dv/ds| = "quasi-steady-state" cornering
SMOOTH_WINDOW = 9

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--track", required=True, help="Track slug matching data/<slug>_track_geometry.csv")
args = parser.parse_args()
slug = args.track.lower()

geo = pd.read_csv(f"data/{slug}_track_geometry.csv")
ref = pd.read_csv(f"data/{slug}_reference_lap.csv")
if not (geo["Distance"].values == ref["Distance"].values).all():
    raise ValueError(f"{slug}: track_geometry.csv and reference_lap.csv Distance axes don't match")

s = geo["Distance"].values
x, y = geo["X"].values, geo["Y"].values
v = ref["Speed"].values / 3.6

x_s = savgol_filter(x, SMOOTH_WINDOW, 3, mode="wrap")
y_s = savgol_filter(y, SMOOTH_WINDOW, 3, mode="wrap")
v_s = savgol_filter(v, SMOOTH_WINDOW, 3, mode="wrap")

dx = np.gradient(x_s, s)
dy = np.gradient(y_s, s)
ddx = np.gradient(dx, s)
ddy = np.gradient(dy, s)
kappa = (dx * ddy - dy * ddx) / np.clip(dx**2 + dy**2, 1e-9, None) ** 1.5
dvds = np.gradient(v_s, s)

dvds_thresh = np.percentile(np.abs(dvds), DVDS_PERCENTILE)
mask = (
    (np.abs(dvds) <= dvds_thresh)
    & (np.abs(kappa) >= MIN_CURVATURE)
    & (v_s * 3.6 >= MIN_APEX_SPEED_KMH)
)

c_l_samples = []
for vi, ki in zip(v_s[mask], np.abs(kappa[mask])):
    a_lat = vi**2 * ki
    c_l_implied = (a_lat / MU_LAT - G) / vi**2
    if c_l_implied > 0:
        c_l_samples.append(c_l_implied)

if not c_l_samples:
    raise ValueError(
        f"{slug}: no qualifying quasi-steady-state cornering points found -- "
        "can't derive c_l for this track (check MIN_APEX_SPEED_KMH/MIN_CURVATURE against "
        "its real corner speeds, or fall back to the 3-tier system in main.rs)"
    )
c_l = float(np.median(c_l_samples))

v_max = np.percentile(v_s, 99.5)
c_d_drs = P_ENGINE / v_max**3
c_d = c_d_drs / DRS_DRAG_REDUCTION

out_path = f"data/{slug}_aero_params.csv"
pd.DataFrame({"c_l": [c_l], "c_d": [c_d]}).to_csv(out_path, index=False)
print(f"{slug}: n_samples={len(c_l_samples)} c_l={c_l:.5f} c_d={c_d:.5f} v_max={v_max*3.6:.1f} km/h -> {out_path}")
