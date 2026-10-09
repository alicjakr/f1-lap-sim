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

import sys
from pathlib import Path

import fastf1
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_pole_times import TRACKS  # noqa: E402  slug -> official 2018 EventName

MU_LAT = 1.6
G = 9.81
P_ENGINE = 1015.0
DRS_DRAG_REDUCTION = 0.88  # matches api.rs's DRS_DRAG_REDUCTION

MIN_APEX_SPEED_KMH = 150.0
MIN_CURVATURE = 0.008
DVDS_PERCENTILE = 25  # bottom 25% of |dv/ds| = "quasi-steady-state" cornering
SMOOTH_WINDOW = 9
MIN_SAMPLES_WARN = 10  # below this, c_l rests on too few apexes to be trustworthy

# Pooling across the field multiplies the sample count, but only laps where the driver was
# actually pushing carry information about the limit: the estimator inverts
# a_lat = mu_lat*(G + c_l*v^2), which assumes the car is AT the lateral limit. A cruising or
# out-lap still produces "quasi-steady cornering" points, just below the limit, and those
# drag the estimate down. 107% of the session's best is F1's own cutoff for a representative
# qualifying lap, so it is the natural filter.
LAP_CUTOFF = 1.07

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--track", required=True, help="Track slug matching data/<slug>_track_geometry.csv")
parser.add_argument("--reference-lap-only", action="store_true",
                    help="Derive c_l from the exported reference lap alone (the pre-pooling "
                         "behaviour), for comparison")
parser.add_argument("--year", type=int, default=2018)
args = parser.parse_args()
slug = args.track.lower()

def c_l_samples_from_lap(s_axis, x, y, v):
    """Implied c_l at every quasi-steady-state apex of one lap.

    Inverts the friction ellipse's lateral term, a_lat = mu_lat*(G + c_l*v^2), at points
    where the car is cornering at roughly constant speed. A negative implied c_l means the
    point produced less lateral acceleration than mechanical grip alone allows -- the driver
    was not at the limit there, so it says nothing about downforce and is dropped rather
    than dragging the estimate down."""
    # np.gradient divides by the spacing, so a repeated Distance sample (which FastF1's
    # telemetry does contain on some laps) produces a division by zero and poisons the whole
    # curvature array with inf/nan.
    if len(s_axis) < SMOOTH_WINDOW + 1 or np.any(np.diff(s_axis) <= 0):
        return np.empty(0)
    x_s = savgol_filter(x, SMOOTH_WINDOW, 3, mode="wrap")
    y_s = savgol_filter(y, SMOOTH_WINDOW, 3, mode="wrap")
    v_s = savgol_filter(v, SMOOTH_WINDOW, 3, mode="wrap")

    dx = np.gradient(x_s, s_axis)
    dy = np.gradient(y_s, s_axis)
    ddx = np.gradient(dx, s_axis)
    ddy = np.gradient(dy, s_axis)
    kappa = (dx * ddy - dy * ddx) / np.clip(dx**2 + dy**2, 1e-9, None) ** 1.5
    dvds = np.gradient(v_s, s_axis)

    mask = (
        (np.abs(dvds) <= np.percentile(np.abs(dvds), DVDS_PERCENTILE))
        & (np.abs(kappa) >= MIN_CURVATURE)
        & (v_s * 3.6 >= MIN_APEX_SPEED_KMH)
    )
    vi = v_s[mask]
    a_lat = vi**2 * np.abs(kappa[mask])
    implied = (a_lat / MU_LAT - G) / np.clip(vi**2, 1e-9, None)
    return implied[implied > 0]


geo = pd.read_csv(f"data/{slug}_track_geometry.csv")
ref = pd.read_csv(f"data/{slug}_reference_lap.csv")
if not (geo["Distance"].values == ref["Distance"].values).all():
    raise ValueError(f"{slug}: track_geometry.csv and reference_lap.csv Distance axes don't match")

s = geo["Distance"].values
v = ref["Speed"].values / 3.6
v_s = savgol_filter(v, SMOOTH_WINDOW, 3, mode="wrap")

if args.reference_lap_only:
    c_l_samples = c_l_samples_from_lap(s, geo["X"].values, geo["Y"].values, v)
    n_laps = 1
else:
    # c_l is a property of the car at this circuit, not of any one lap, so there is no reason
    # to estimate it from a single one. Two laps of the same car moved the estimate by 16%
    # (see results/DISCUSSION.md V9), which is the variance this is meant to beat down.
    fastf1.Cache.enable_cache("cache")
    session = fastf1.get_session(args.year, TRACKS[slug], "Q")
    session.load(telemetry=True, weather=False, messages=False)
    best = session.laps.pick_fastest()["LapTime"].total_seconds()
    pooled, n_laps, skipped = [], 0, 0
    for _, lap in session.laps.iterlaps():
        lap_time = lap["LapTime"]
        if pd.isna(lap_time) or lap_time.total_seconds() > best * LAP_CUTOFF:
            continue
        try:
            tel = lap.get_telemetry()[["Distance", "X", "Y", "Speed"]].dropna()
        except Exception:
            skipped += 1
            continue
        tel = tel.sort_values("Distance").drop_duplicates(subset="Distance")
        if len(tel) < SMOOTH_WINDOW + 1:
            skipped += 1
            continue
        got = c_l_samples_from_lap(tel["Distance"].values, tel["X"].values / 10.0,
                                   tel["Y"].values / 10.0, tel["Speed"].values / 3.6)
        if len(got):
            pooled.append(got)
            n_laps += 1
    if skipped:
        print(f"  ({skipped} lap(s) skipped: telemetry unavailable)")
    c_l_samples = np.concatenate(pooled) if pooled else np.empty(0)

if not len(c_l_samples):
    raise ValueError(
        f"{slug}: no qualifying quasi-steady-state cornering points found -- "
        "can't derive c_l for this track (check MIN_APEX_SPEED_KMH/MIN_CURVATURE against "
        "its real corner speeds, or fall back to the 3-tier system in api.rs)"
    )
c_l = float(np.median(c_l_samples))
if len(c_l_samples) < MIN_SAMPLES_WARN:
    print(
        f"WARNING: {slug}: c_l derived from only {len(c_l_samples)} qualifying apex "
        f"sample(s); treat this track's aero (and so its lap time) as weakly determined."
    )

v_max = np.percentile(v_s, 99.5)
c_d_drs = P_ENGINE / v_max**3
c_d = c_d_drs / DRS_DRAG_REDUCTION

out_path = f"data/{slug}_aero_params.csv"
pd.DataFrame({"c_l": [c_l], "c_d": [c_d]}).to_csv(out_path, index=False)
iqr = np.percentile(c_l_samples, 75) / max(np.percentile(c_l_samples, 25), 1e-9)
print(f"{slug}: n_laps={n_laps} n_samples={len(c_l_samples)} c_l={c_l:.5f} "
      f"(p75/p25 spread {iqr:.1f}x) c_d={c_d:.5f} v_max={v_max*3.6:.1f} km/h -> {out_path}")
