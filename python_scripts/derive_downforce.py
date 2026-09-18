"""
Derive per-track (c_l, c_d) aero coefficients from a track's own real FastF1 telemetry,
instead of guessing which of three fixed tiers (api.rs's HIGH/MED/LOW_DOWNFORCE) a circuit
belongs to. See results/DISCUSSION.md for why (the 3-tier system's validation gap) and the
full validation history, including the rejected joint mu_lat/c_l regression.

Two independent measurements per track, both from the same lap export_track.py already
wrote (data/<slug>_track_geometry.csv's X/Y and data/<slug>_reference_lap.csv's Speed share
the same Distance axis, unlike the OSM boundary data -- no cross-axis interpolation needed):

1. c_d from top speed: power/drag balance (P = c_d_drs * m * v^3) at the 99.5th percentile
   of smoothed speed (not the raw max, to avoid one noisy sample swinging a cubic
   relationship), assuming DRS is open there.

2. c_l from real apex lateral acceleration: the friction ellipse says a_lat = mu_lat*(G +
   c_l*v^2) at a point of pure cornering. Every point where |dv/ds| falls in the bottom 25th
   percentile for that lap counts as "quasi-steady-state cornering" (captures whole apex
   plateaus, not just single local-minimum points -- an earlier attempt using only exact
   minima starved several tracks of samples). Filtered to real corners (curvature >=
   MIN_CURVATURE) fast enough that aero is a meaningful fraction of total grip (>=
   MIN_APEX_SPEED_KMH). Takes the per-track median of the implied c_l.

X/Y and Speed are lightly smoothed (Savitzky-Golay, ~9-sample window) before differentiating
for curvature and dv/ds, to avoid spurious "corners" and unstable derivatives from raw
telemetry noise.

mu_lat stays a fixed global constant rather than being derived per track -- tried, rejected;
see DISCUSSION.md for why the joint regression on v^2 fails (outlier leverage from
misclassified straight-line points).

Usage:
  python derive_downforce.py --track monaco

Requires data/<slug>_track_geometry.csv and data/<slug>_reference_lap.csv (export_track.py).
Outputs data/<slug>_aero_params.csv (c_l, c_d), read by src/api.rs (track::load_aero_params)
in place of the 3-tier fallback when present.
"""

import argparse

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

MU_LAT = 1.6
G = 9.81
P_ENGINE = 1015.0
DRS_DRAG_REDUCTION = 0.88  # matches api.rs's DRS_DRAG_REDUCTION

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
        "its real corner speeds, or fall back to the 3-tier system in api.rs)"
    )
c_l = float(np.median(c_l_samples))

v_max = np.percentile(v_s, 99.5)
c_d_drs = P_ENGINE / v_max**3
c_d = c_d_drs / DRS_DRAG_REDUCTION

out_path = f"data/{slug}_aero_params.csv"
pd.DataFrame({"c_l": [c_l], "c_d": [c_d]}).to_csv(out_path, index=False)
print(f"{slug}: n_samples={len(c_l_samples)} c_l={c_l:.5f} c_d={c_d:.5f} v_max={v_max*3.6:.1f} km/h -> {out_path}")
