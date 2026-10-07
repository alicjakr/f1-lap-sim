"""
Export track boundaries from the TUM racetrack-database, as an alternative to the
OpenStreetMap path in export_osm_boundaries.py.

Why: OSM has no track widths. Every circuit exported through export_osm_boundaries.py falls
back to DEFAULT_LANES * LANE_WIDTH_M (7 m, invented), and the resulting corridor is centred
on the FastF1 driven line -- which is itself a racing line. The solver therefore gets room
to move *inside* an apex the driver had already clipped, and spends the same track width
twice. That is the single biggest error in `racingline` mode (see results/DISCUSSION.md V7).

TUMFTM/racetrack-database (LGPL-3.0), from the Chair of Automotive Technology at the
Technical University of Munich (contact: Alexander Heilmeier; the satellite width-extraction
algorithm is the work of Andressa de Paula Suiti), publishes per circuit a full-scale
centerline with per-point distances to each edge:

    x_m, y_m, w_tr_right_m, w_tr_left_m

Centerlines came from OSM GPS traces; the widths were extracted from satellite imagery, so
they are measured rather than assumed. Crucially the centerline is the *track* centre, not a
driven line, so the offset between it and our reference lap is real information: it says
where in the track the driver actually was, and that makes the corridor asymmetric -- near
zero to the inside at an apex, wide to the outside.

Two caveats their README states, both of which matter here. The centerlines are smoothed, so
they no longer lie perfectly in the middle of the track -- which is part of why registration
lands at a 3-4 m residual rather than nearer zero. And source quality varies by location,
since it depends on the GPS traces and satellite imagery available for that circuit, so the
widths are not uniformly accurate across the 17.

Nothing is committed back to that repository; this only downloads from it. The CSVs are
cached under cache/tum_tracks/ and data/ is gitignored, so their files are never
redistributed from here.

Usage:
  python export_tum_boundaries.py --track suzuka
  python export_tum_boundaries.py --track monza --plot-check

Requires data/<slug>_track_geometry.csv (export_track.py first). Writes
data/<slug>_track_boundaries.csv in the same format the Rust side already reads, so the
solver needs no changes.

Not every circuit is in the database -- Baku, Monaco, Paul Ricard and Singapore are absent,
and keep whatever export_osm_boundaries.py produced for them.
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import requests
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
from export_osm_boundaries import despike_offset, register  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
CACHE = REPO / "cache" / "tum_tracks"
BASE_URL = "https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks"

# Our track slug -> the database's file name. Only circuits that exist in both.
TUM_FILES = {
    "bahrain": "Sakhir",
    "catalunya": "Catalunya",
    "cota": "Austin",
    "hockenheim": "Hockenheim",
    "hungaroring": "Budapest",
    "interlagos": "SaoPaulo",
    "melbourne": "Melbourne",
    "mexico": "MexicoCity",
    "montreal": "Montreal",
    "monza": "Monza",
    "shanghai": "Shanghai",
    "silverstone": "Silverstone",
    "sochi": "Sochi",
    "spa": "Spa",
    "redbullring": "Spielberg",
    "suzuka": "Suzuka",
    "yasmarina": "YasMarina",
}


def fetch_track(name):
    """Download one circuit's centerline+widths, caching it so reruns need no network."""
    CACHE.mkdir(parents=True, exist_ok=True)
    cached = CACHE / f"{name}.csv"
    if not cached.exists():
        url = f"{BASE_URL}/{name}.csv"
        print(f"Downloading {url}")
        response = requests.get(url, timeout=60)
        response.raise_for_status()
        cached.write_text(response.text)
    # The published files carry a commented header line.
    frame = pd.read_csv(cached, comment="#", header=None,
                        names=["x_m", "y_m", "w_tr_right_m", "w_tr_left_m"])
    return frame.apply(pd.to_numeric, errors="coerce").dropna().reset_index(drop=True)


def unit_tangents(points):
    """Unit tangent at each point of a closed polyline, by central difference."""
    tangent = np.gradient(points, axis=0)
    return tangent / np.clip(np.linalg.norm(tangent, axis=1)[:, None], 1e-9, None)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--track", required=True, help="Track slug matching data/<slug>_track_geometry.csv")
    parser.add_argument("--jump-threshold", type=float, default=14.0, help="despike_offset: see export_osm_boundaries.py")
    parser.add_argument("--outlier-threshold", type=float, default=10.0, help="despike_offset: see export_osm_boundaries.py")
    parser.add_argument("--hysteresis-high", type=float, default=20.0, help="despike_offset: see export_osm_boundaries.py")
    parser.add_argument("--hysteresis-low", type=float, default=10.0, help="despike_offset: see export_osm_boundaries.py")
    args = parser.parse_args()
    slug = args.track.lower()

    if slug not in TUM_FILES:
        raise SystemExit(
            f"{slug!r} is not in the TUM database ({', '.join(sorted(TUM_FILES))}). "
            "Use export_osm_boundaries.py for this circuit."
        )

    track = fetch_track(TUM_FILES[slug])
    centre = track[["x_m", "y_m"]].values
    w_left = track["w_tr_left_m"].values
    w_right = track["w_tr_right_m"].values
    print(f"{len(centre)} centerline points, mean width "
          f"{(w_left + w_right).mean():.1f} m (left {w_left.mean():.1f}, right {w_right.mean():.1f})")

    reference = pd.read_csv(REPO / "data" / f"{slug}_track_geometry.csv")
    ref_pts = reference[["X", "Y"]].values
    s = reference["Distance"].values

    # Both frames are metric and unscaled, so a rigid transform is all that's needed -- the
    # same ICP the OSM path uses.
    registered, residual = register(centre, ref_pts)
    print(f"Registered to the FastF1 frame: mean residual {residual:.2f} m")
    if residual > 15.0:
        print("WARNING: residual is far above the few metres a real centerline-vs-racing-line "
              "gap should give. Treat this track's output as suspect.")

    ref_tangent = unit_tangents(ref_pts)
    ref_normal = np.column_stack([-ref_tangent[:, 1], ref_tangent[:, 0]])  # left of travel

    tree = cKDTree(registered)
    _, nearest = tree.query(ref_pts)

    # The database may store its centerline in the opposite direction of travel, which would
    # mirror left and right. Compare tangents at the matched points and swap if so.
    centre_tangent = unit_tangents(registered)
    if np.mean(np.sum(centre_tangent[nearest] * ref_tangent, axis=1)) < 0:
        print("Centerline runs opposite to the driven lap; swapping left/right widths")
        w_left, w_right = w_right, w_left

    # How far our driven line sits from the track centre, positive to the left of travel.
    delta = registered[nearest] - ref_pts
    centre_offset = -(delta[:, 0] * ref_normal[:, 0] + delta[:, 1] * ref_normal[:, 1])
    half_width = (w_left[nearest] + w_right[nearest]) / 2.0

    centre_offset, half_width, n_despiked = despike_offset(
        s, centre_offset, half_width,
        jump_threshold=args.jump_threshold, outlier_threshold=args.outlier_threshold,
        hysteresis_high=args.hysteresis_high, hysteresis_low=args.hysteresis_low,
    )
    if n_despiked:
        print(f"Despiked {n_despiked} sample(s) where the nearest-centerline lookup jumped")

    # Distance from the driven line to each edge. This is the asymmetry the OSM path can't
    # supply: at an apex one side goes to nearly zero and the other opens up.
    n_left = half_width - centre_offset
    n_right = -(half_width + centre_offset)

    outside = np.mean((n_left < 0) | (n_right > 0)) * 100
    print(f"Driven line sits {centre_offset.mean():+.2f} m from the track centre on average "
          f"(min {centre_offset.min():+.2f}, max {centre_offset.max():+.2f})")
    print(f"Driven line outside the measured track at {outside:.1f}% of samples"
          + ("  <-- registration or data problem" if outside > 5 else ""))

    out_path = REPO / "data" / f"{slug}_track_boundaries.csv"
    pd.DataFrame({
        "s": s,
        "x": ref_pts[:, 0],
        "y": ref_pts[:, 1],
        "n_left": n_left,
        "n_right": n_right,
    }).to_csv(out_path, index=False)
    print(f"Exported {len(s)} boundary samples -> {out_path}")
    print(f"Corridor width {np.mean(n_left - n_right):.1f} m "
          f"(the OSM path assumed a flat 7.0 m for every circuit)")


if __name__ == "__main__":
    main()
