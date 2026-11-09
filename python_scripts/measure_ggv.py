"""
Measure the car's real grip envelope from pooled qualifying telemetry and test which
model shape fits it: the solver's lumped  a_lat = mu*(G + c_l*v^2)  or a load-sensitive tyre,
a_lat = mu(N)*N  with  N = G + c_l*v^2  (per unit mass) and  mu(N) = mu0 - k*(N - G).

Why: derive_downforce.py estimates c_l as a *median* of quasi-steady corner samples, but the
grip equation describes a ceiling the driver is rarely exactly at, so the median sits under
it (DISCUSSION.md V11). The envelope itself -- a high quantile per speed bin -- targets the
ceiling directly, and its shape in v says whether a linear-in-v^2 model can be right at all.

Curvature comes from the same smoother the solver uses (track.rs: periodic cubic P-spline,
12 m knots, lambda=1, re-implemented below), fitted to each lap's own X/Y, not from
differentiating raw telemetry: V11 showed that inflates a_lat with artefacts (up to 42 g),
and a high quantile is exactly the statistic that picks those up. Reusing the *reference*
lap's curvature for every lap was tried first and fails the other way -- aligning corner
peaks across laps is off by tens of metres, and a 99th-percentile hull reads that as 6-9 g.

Outputs a per-speed-bin table (lateral, braking, traction), then least-squares fits of both
lateral models to the bin values, reported with their residuals. Nothing is written to
data/ or read by the solver; this is a measurement, not a calibration.

What the three analyses can and cannot tell you (all measured, 21 circuits, 2018 Q):

  --apex      The cleanest. Uses only data/ (no network): the exported reference lap and its
              own geometry share one Distance axis, which is also the spline's parameter, so
              curvature at the driver's apex speed minimum needs no alignment between laps.
              Reports the lateral g the driver pulled at each apex relative to the solver's
              own grip ceiling for that circuit, by apex speed. A ratio above 1 means the
              driver's *median* apex already exceeds the model's *maximum*.
  (default)   Pooled-lap lateral envelope by speed bin. Not usable above about the median:
              every lap needs its own curvature, so artefacts fill the tail a high quantile
              reads (p75 and up reads 5-14 g). Kept because it shows that, not as a result.
  (default)   Braking envelope. Curvature-free but not a tyre limit at high speed -- it is a
              hump (peaks near 77 m/s) set by the brake system and driver, so it cannot
              discriminate grip models either.

Usage:
  python python_scripts/measure_ggv.py --apex
  python python_scripts/measure_ggv.py --track monza
  python python_scripts/measure_ggv.py --all --quantile 0.99
"""

import argparse
import sys
from pathlib import Path

import fastf1
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.sparse import csr_matrix, eye as sparse_eye
from scipy.sparse.linalg import spsolve
from scipy.signal import savgol_filter

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_pole_times import TRACKS  # noqa: E402  slug -> official 2018 EventName

REPO = Path(__file__).resolve().parent.parent
G = 9.81
GRID_DS_M = 5.0          # same uniform grid as derive_downforce.py; see its note on FastF1's Distance
SMOOTH_WINDOW = 9
LAP_CUTOFF = 1.07        # F1's own representative-qualifying-lap cutoff
STEADY_A_LON = 1.5       # m/s^2: |v dv/ds| below this counts as quasi-steady cornering
MIN_CURVATURE = 0.004    # 1/m: below this a point is a straight, not a corner
BIN_WIDTH = 5.0          # m/s
MIN_BIN_SAMPLES = 30
# Physical c_l per kg implied by TUM's public F1 parameter set (0.5*rho*(CzA_f+CzA_r)/m with
# rho 1.18, CzA 2.20+2.68 m^2, m 733 kg). Used only to pin the aero term in the load-sensitive
# fit so that fit has the same two free parameters as the lumped one.
C_L_PHYSICAL = 0.5 * 1.18 * (2.20 + 2.68) / 733.0


CONTROL_POINT_SPACING_M = 12.0   # api.rs CONTROL_POINT_SPACING_M
SMOOTHING_LAMBDA = 1.0           # api.rs SMOOTHING_LAMBDA


def _basis(u):
    u2, u3 = u * u, u * u * u
    return np.stack([(1 - u) ** 3 / 6, (3 * u3 - 6 * u2 + 4) / 6,
                     (-3 * u3 + 3 * u2 + 3 * u + 1) / 6, u3 / 6], axis=1)


def _basis_d1(u):
    return np.stack([-(1 - u) ** 2 / 2, (3 * u * u - 4 * u) / 2,
                     (-3 * u * u + 2 * u + 1) / 2, u * u / 2], axis=1)


def _basis_d2(u):
    return np.stack([1 - u, 3 * u - 2, -3 * u + 1, u], axis=1)


def _design(t, h, m, basis):
    """Sparse (len(t) x m) matrix of a periodic cubic B-spline basis, as in track.rs."""
    raw = np.mod(t, m * h) / h
    seg = np.minimum(np.floor(raw).astype(int), m - 1)
    w = basis(raw - seg)
    cols = (seg[:, None] - 1 + np.arange(4)[None, :]) % m
    return csr_matrix((w.ravel(), (np.repeat(np.arange(len(t)), 4), cols.ravel())), shape=(len(t), m))


def pspline_curvature(t, x, y, length, t_query):
    """Signed curvature at t_query of a periodic P-spline fitted to (x, y) sampled at t.

    Same smoother as track.rs::fit_periodic_bspline: m = round(length/12) control points, a
    periodic second-difference penalty on the control polygon with weight lambda."""
    m = int(round(length / CONTROL_POINT_SPACING_M))
    h = length / m
    b = _design(t, h, m, _basis)
    d = sparse_eye(m, format="csr") * -2 + sparse_eye(m, k=1, format="csr") + sparse_eye(m, k=-1, format="csr")
    d = d + sparse_eye(m, k=m - 1, format="csr") + sparse_eye(m, k=-(m - 1), format="csr")
    a = (b.T @ b + SMOOTHING_LAMBDA * (d.T @ d)).tocsc()
    cx, cy = spsolve(a, b.T @ x), spsolve(a, b.T @ y)
    q1, q2 = _design(t_query, h, m, _basis_d1), _design(t_query, h, m, _basis_d2)
    dx, dy = (q1 @ cx) / h, (q1 @ cy) / h
    ddx, ddy = (q2 @ cx) / h**2, (q2 @ cy) / h**2
    return (dx * ddy - dy * ddx) / (dx * dx + dy * dy) ** 1.5


def lap_points(slug, session):
    """Per-lap (v, |a_lat|, a_lon, throttle, brake) samples on a uniform grid."""
    best = session.laps.pick_fastest()["LapTime"].total_seconds()
    length_ref = session.laps.pick_fastest().get_telemetry()["Distance"].max()
    out = []
    for _, lap in session.laps.iterlaps():
        lt = lap["LapTime"]
        if pd.isna(lt) or lt.total_seconds() > best * LAP_CUTOFF:
            continue
        try:
            tel = lap.get_telemetry()[["Distance", "X", "Y", "Speed", "Throttle", "Brake"]].dropna()
        except Exception:
            continue
        tel = tel.sort_values("Distance").drop_duplicates(subset="Distance")
        if len(tel) < SMOOTH_WINDOW + 1 or tel["Distance"].iloc[-1] < 0.9 * length_ref:
            continue
        s = tel["Distance"].values
        grid = np.arange(s[0], s[-1], GRID_DS_M)
        if len(grid) < SMOOTH_WINDOW + 1:
            continue
        v = savgol_filter(np.interp(grid, s, tel["Speed"].values / 3.6), SMOOTH_WINDOW, 3, mode="nearest")
        thr = np.interp(grid, s, tel["Throttle"].values)
        brk = np.interp(grid, s, tel["Brake"].values.astype(float)) > 0.5
        # Closed-loop length as track.rs::total_length: last Distance plus the closing chord.
        # A lap whose first and last position samples are far apart (a telemetry dropout, see
        # export_track.py) gets a meaningless chord, so it is rejected rather than fitted.
        xy = tel[["X", "Y"]].values / 10.0
        chord = float(np.hypot(*(xy[0] - xy[-1])))
        if chord > 20.0:
            continue
        length = float(s[-1]) + chord
        k = pspline_curvature(s, xy[:, 0], xy[:, 1], length, grid)
        a_lon = v * np.gradient(v, GRID_DS_M)
        out.append(np.column_stack([v, np.abs(k) * v**2, a_lon, np.abs(k), thr, brk]))
    return np.vstack(out) if out else np.empty((0, 6))


def lateral_models(v, p, kind):
    if kind == "lumped":          # mu, c_l free
        mu, c = p
        return mu * (G + c * v**2)
    if kind == "sensitive":       # mu0, k free; aero term pinned to the physical value
        mu0, k = p
        n = G + C_L_PHYSICAL * v**2
        return (mu0 - k * (n - G)) * n
    mu0, k, c = p                 # "sensitive_free": c_l free as well
    n = G + c * v**2
    return (mu0 - k * (n - G)) * n


def fit(v, a, kind, p0):
    res = least_squares(lambda p: lateral_models(v, p, kind) - a, p0)
    rms = float(np.sqrt(np.mean(res.fun**2)))
    return res.x, rms


MU_LAT = 1.6  # api.rs load_car_params; duplicated here for the same reason derive_downforce.py does it


def apex_grip_by_speed():
    """Lateral g at each driver apex, relative to the solver's grip ceiling there."""
    from scipy.signal import find_peaks
    rows = []
    for path in sorted((REPO / "data").glob("*_track_geometry.csv")):
        slug = path.name[: -len("_track_geometry.csv")]
        ref_path, aero_path = REPO / "data" / f"{slug}_reference_lap.csv", REPO / "data" / f"{slug}_aero_params.csv"
        if not (ref_path.exists() and aero_path.exists()):
            continue
        g, r = pd.read_csv(path), pd.read_csv(ref_path)
        if not (g["Distance"].values == r["Distance"].values).all():
            continue
        c_l = float(pd.read_csv(aero_path)["c_l"].iloc[0])
        s, xy, spd = g["Distance"].values, g[["X", "Y"]].values, r["Speed"].values / 3.6
        keep = np.append(np.diff(s) > 0, True)
        s, xy, spd = s[keep], xy[keep], spd[keep]
        length = float(s[-1]) + float(np.hypot(*(xy[0] - xy[-1])))
        grid = np.arange(s[0], s[-1], GRID_DS_M)
        kappa = np.abs(pspline_curvature(s, xy[:, 0], xy[:, 1], length, grid))
        v = savgol_filter(np.interp(grid, s, spd), 5, 2)
        for i in find_peaks(-v, prominence=4.0)[0]:
            a_lat = v[i] ** 2 * kappa[i]
            rows.append((slug, v[i], a_lat / G, a_lat / (MU_LAT * (G + c_l * v[i] ** 2))))
    d = pd.DataFrame(rows, columns=["track", "v", "g", "vs_model"])
    d["band"] = pd.cut(d["v"], [0, 20, 30, 40, 50, 60, 100])
    print(f"{len(d)} apexes over {d['track'].nunique()} circuits; lateral g at the driver's speed minimum")
    print("  vs_model = driver's a_lat / solver's ceiling mu_lat*(G + c_l*v^2) for that circuit")
    t = d.groupby("band", observed=True).agg(
        n=("g", "size"), g_median=("g", "median"), ratio_p25=("vs_model", lambda x: x.quantile(0.25)),
        ratio_median=("vs_model", "median"), ratio_p75=("vs_model", lambda x: x.quantile(0.75)))
    print(t.round(2).to_string())
    print(f"  overall: median {d['g'].median():.2f} g; {100 * (d['g'] > 6).mean():.0f}% of apexes above 6 g "
          f"(curvature artefacts: treat the tail, not the median, as suspect)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--apex", action="store_true", help="apex lateral g vs the solver's ceiling, from data/ only")
    ap.add_argument("--track", help="one slug, e.g. monza")
    ap.add_argument("--all", action="store_true", help="pool every circuit in validate_pole_times.TRACKS")
    ap.add_argument("--quantile", type=float, nargs="+", default=[0.5, 0.75, 0.9, 0.95, 0.99],
                    help="lateral-envelope quantiles per speed bin; the largest is also used for "
                         "braking and traction (default: 0.5 0.75 0.9 0.95 0.99)")
    ap.add_argument("--year", type=int, default=2018)
    ap.add_argument("--cache", help="npz file: load pooled samples from it if present, else write them there "
                                    "(skips the ~3 min telemetry pass when re-slicing the same data)")
    args = ap.parse_args()
    if args.apex:
        apex_grip_by_speed()
        return
    if not args.track and not args.all:
        ap.error("pass --apex, --track <slug> or --all")

    if args.cache and Path(args.cache).exists():
        pts = np.load(args.cache)["pts"]
    else:
        slugs = sorted(TRACKS) if args.all else [args.track.lower()]
        fastf1.Cache.enable_cache(str(REPO / "cache"))
        chunks = []
        for slug in slugs:
            try:
                session = fastf1.get_session(args.year, TRACKS[slug], "Q")
                session.load(telemetry=True, weather=False, messages=False)
                pts = lap_points(slug, session)
            except Exception as e:  # one circuit with no cached data should not stop a sweep
                print(f"{slug}: skipped ({type(e).__name__}: {e})")
                continue
            print(f"{slug}: {len(pts)} samples")
            chunks.append(pts)
        pts = np.vstack(chunks)
        if args.cache:
            np.savez_compressed(args.cache, pts=pts)
    v, a_lat, a_lon, kap, thr, brk = pts.T

    edges = np.arange(10.0, 95.0, BIN_WIDTH)
    qs = sorted(args.quantile)
    q_edge = qs[-1]
    lat = {q: ([], []) for q in qs}
    print(f"\nEnvelope per speed bin, in g (a/9.81); braking and traction at quantile {q_edge}")
    print(f"{'v m/s':>7} {'n_lat':>6} " + " ".join(f"lat p{int(q * 100):>2}" for q in qs)
          + f" | {'n_brk':>5} {'brake':>6} | {'n_acc':>5} {'accel':>6}")
    for lo in edges:
        m = (v >= lo) & (v < lo + BIN_WIDTH)
        # Lateral: a real corner, held at roughly constant speed.
        mc = m & (kap >= MIN_CURVATURE) & (np.abs(a_lon) < STEADY_A_LON)
        # Braking: brake pedal on, nearly straight, so the tyres are not also cornering.
        mb = m & (brk > 0.5) & (kap < 0.002)
        # Traction: full throttle on a straight. Raw a_lon includes drag; reported as measured.
        ma = m & (thr >= 99) & (kap < 0.002) & (a_lon > 0)
        row = f"{lo + BIN_WIDTH / 2:7.1f} {mc.sum():6d} "
        for q in qs:
            if mc.sum() >= MIN_BIN_SAMPLES:
                val = np.quantile(a_lat[mc], q)
                lat[q][0].append(lo + BIN_WIDTH / 2)
                lat[q][1].append(val)
                row += f"{val / G:7.2f} "
            else:
                row += f"{'-':>7} "
        row += f"| {mb.sum():5d} " + (f"{-np.quantile(a_lon[mb], 1 - q_edge) / G:6.2f}" if mb.sum() >= MIN_BIN_SAMPLES else f"{'-':>6}")
        row += f" | {ma.sum():5d} " + (f"{np.quantile(a_lon[ma], q_edge) / G:6.2f}" if ma.sum() >= MIN_BIN_SAMPLES else f"{'-':>6}")
        print(row)

    print("\nLeast-squares fits to the lateral envelope at each quantile (rms in m/s^2):")
    print(f"  aero for 'sensitive' pinned at c_l={C_L_PHYSICAL:.5f}; solver today: mu_lat=1.6, c_l median ~0.0052")
    for q in qs:
        lv, la = np.array(lat[q][0]), np.array(lat[q][1])
        if len(lv) < 4:
            print(f"  p{int(q * 100)}: too few populated bins")
            continue
        (mu, c), ra = fit(lv, la, "lumped", [1.6, 0.005])
        (mu0, k), rb = fit(lv, la, "sensitive", [1.9, 3e-4])
        (mu0f, kf, cf), rc = fit(lv, la, "sensitive_free", [1.9, 3e-4, 0.004])
        print(f"  p{int(q * 100):<2} lumped mu={mu:.2f} c_l={c:.5f} rms={ra:.2f} | "
              f"sensitive mu0={mu0:.2f} k={k:.1e} rms={rb:.2f} | "
              f"sensitive+c_l mu0={mu0f:.2f} k={kf:.1e} c_l={cf:.5f} rms={rc:.2f}")

    brake_fit(v, a_lon, brk, kap, q_edge)


def brake_fit(v, a_lon, brk, kap, q):
    """Fit grip models to the braking envelope, which needs no curvature and so is clean.

    The tyres supply  a_brake - c_d*v^2  (drag decelerates the car for free), so that is what
    is compared with mu*(G + c_l*v^2) or the load-sensitive form. c_d is fixed at a typical
    derived value; it is a small term here and not what is being tested."""
    cd = 0.0014
    vs, a = [], []
    for lo in np.arange(20.0, 95.0, BIN_WIDTH):
        m = (v >= lo) & (v < lo + BIN_WIDTH) & (brk > 0.5) & (kap < 0.002)
        if m.sum() >= MIN_BIN_SAMPLES:
            vc = lo + BIN_WIDTH / 2
            vs.append(vc)
            a.append(-np.quantile(a_lon[m], 1 - q) - cd * vc**2)
    vs, a = np.array(vs), np.array(a)
    print(f"\nBraking envelope (quantile {q}, tyre share = decel - c_d*v^2 with c_d={cd}):")
    for kind, p0, label in (("lumped", [1.6, 0.005], "lumped      (mu, c_l free)"),
                            ("sensitive", [1.9, 3e-4], "sensitive   (mu0, k; c_l pinned)"),
                            ("sensitive_free", [1.9, 3e-4, 0.004], "sensitive+c (mu0, k, c_l free)")):
        p, rms = fit(vs, a, kind, p0)
        print(f"  {label:<34} params={np.round(p, 5).tolist()} rms={rms:.3f} m/s^2")
        print("     residual model-data by bin: " + " ".join(f"{d:+.2f}" for d in lateral_models(vs, p, kind) - a))
    print("     bins v (m/s):               " + " ".join(f"{x:5.1f}" for x in vs))


if __name__ == "__main__":
    main()
