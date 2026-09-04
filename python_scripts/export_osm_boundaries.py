"""
Export track boundary (left/right edge) geometry from OpenStreetMap, registered against
the FastF1-derived reference line. Uses OSM's road geometry (the actual paved surface)
rather than the driven-line spread of FastF1 telemetry -- see results/DISCUSSION.md for
why that alternative was rejected (driver-line convergence isn't track width).

Pipeline, in order:

1. Two ways circuits show up in OSM. Most circuits have a `type=circuit` relation listing
   constituent road/raceway ways, which carry `lanes` tags -- width imputed as
   lanes * LANE_WIDTH_M (OSM's own recommended imputation when width is untagged), falling
   back to DEFAULT_LANES for ways without even a `lanes` tag. fetch_segments_from_relation
   matches the relation's `name` against both the local-language and `name:en` tags.
   --bbox (querying highway=raceway ways in a lat/lon box) is a fallback of last resort for
   a circuit with no relation: no lanes tags (one constant FALLBACK_WIDTH_M throughout) and
   riskier, since a permanent circuit's grounds can contain unrelated `sport=motor` raceways
   with nothing to exclude them by. Prefer --relation-name; reach for --bbox only once
   you've confirmed via Overpass that no relation exists.
2. A relation's member order isn't reliably sequential along the route, so ways are
   reassembled by greedy nearest-endpoint chaining regardless of source (relation or bbox).
3. OSM coordinates are lat/lon; FastF1's X/Y are that session's own local positioning
   frame. Registered via ICP (brute-force initial rotation search, then iterated
   closest-point matching + Kabsch rigid-transform fitting) between the OSM road centerline
   and the FastF1 driven line. Mean registration residual is the data-quality signal to
   check on a new track -- ~5-10 m is expected (a genuine racing-line-vs-road-centerline
   gap); well outside that range signals a bad relation match, not something to average
   over silently.
4. Track width is converted to left/right *offsets from our own reference line* (the
   FastF1 driven line, not necessarily centered in the road) -- what the racing-line
   optimizer needs. n > 0 is left of the driving direction, n < 0 is right;
   n_right[i] <= n[i] <= n_left[i].
5. despike_offset() cleans up two shapes of nearest-point matching error (see its own
   docstring). Its defaults are a cross-track compromise that isn't tight enough for every
   track -- override per track with --jump-threshold/--outlier-threshold/
   --hysteresis-high/--hysteresis-low when a track's own deviation percentiles show a real
   leftover artifact under the shared defaults, e.g.:
     redbullring: --jump-threshold 10 --outlier-threshold 6 --hysteresis-high 8 --hysteresis-low 5
     yasmarina:   --jump-threshold 13 --outlier-threshold 8 --hysteresis-high 13 --hysteresis-low 8
   Validate any override via racing-line mode converging with a negative-or-near-zero delta
   against the fixed-line lap time.
6. Singapore remains unresolved -- confirmed to be real track geometry hitting the
   solver's steering-rate/small-angle model limit (optimal.rs), not an OSM data problem.
   See DISCUSSION.md for the diagnosis. Left as a known limitation alongside
   Bahrain/Catalunya/Sochi/Mexico.

Usage:
  python export_osm_boundaries.py --track singapore --relation-name "Marina Bay"
  python export_osm_boundaries.py --track suzuka --bbox 34.83,136.53,34.85,136.55

Requires data/<slug>_track_geometry.csv to already exist (run export_track.py first).
"""

import argparse
import math

import numpy as np
import pandas as pd
import requests
from scipy.spatial import cKDTree

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
LANE_WIDTH_M = 3.5
DEFAULT_LANES = 2.0
FALLBACK_WIDTH_M = 13.0  # used only when no lanes/width tag exists at all (permanent circuits)
EARTH_RADIUS_M = 6371000.0
# Overpass's server rejects requests's default headers (406) -- match what curl sends.
HEADERS = {"User-Agent": "curl/8.7.1", "Accept": "*/*"}


def haversine_m(p1, p2):
    lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
    lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def overpass_query(query):
    resp = requests.post(OVERPASS_URL, data={"data": query}, headers=HEADERS, timeout=90)
    resp.raise_for_status()
    return resp.json()


def way_width(tags):
    if "width" in tags:
        return float(tags["width"])
    if "lanes" in tags:
        return float(tags["lanes"]) * LANE_WIDTH_M
    return None  # caller decides the right fallback for its source


def fetch_segments_from_relation(relation_name):
    """Street circuits: a type=circuit relation lists constituent public-road ways, which
    carry lanes tags. Returns None if no matching relation exists (caller falls back to bbox).

    Matches against both `name` and `name:en` (unioned) -- some circuits (Suzuka: relation
    284570, "鈴鹿サーキット") only carry the searched-for English name in `name:en`,
    with `name` in the local language. Searching `name` alone silently finds nothing for
    these and falls through to the much noisier --bbox path."""
    query = (
        f'[out:json][timeout:60];'
        f'(relation["type"="circuit"]["name"~"{relation_name}",i];'
        f'relation["type"="circuit"]["name:en"~"{relation_name}",i];);'
        f'out body;>;out geom;'
    )
    data = overpass_query(query)
    relations = [e for e in data["elements"] if e["type"] == "relation"]
    if not relations:
        return None
    if len(relations) > 1:
        names = [r["tags"].get("name") for r in relations]
        raise ValueError(f"Ambiguous match for '{relation_name}': {names}. Narrow the search string.")
    relation = relations[0]
    ways = {e["id"]: e for e in data["elements"] if e["type"] == "way"}
    print(f"Found relation {relation['id']}: {relation['tags'].get('name')}")

    segments = []
    for member in relation["members"]:
        if member["type"] != "way" or member.get("role") in ("pitlane", "pit_lane"):
            continue
        way = ways.get(member["ref"])
        if not way or "geometry" not in way or len(way["geometry"]) < 2:
            continue
        pts = [(pt["lat"], pt["lon"]) for pt in way["geometry"]]
        width = way_width(way["tags"])
        if width is None:
            width = DEFAULT_LANES * LANE_WIDTH_M
        segments.append((pts, width))
    return segments


def fetch_segments_from_bbox(bbox):
    """Permanent circuits without a relation: query highway=raceway ways directly within a
    lat/lon bounding box, excluding the pit lane by name. No lanes/width tags exist for these
    (checked on Suzuka) so every segment gets FALLBACK_WIDTH_M -- no per-point width signal,
    unlike the relation path."""
    lat_min, lon_min, lat_max, lon_max = bbox
    query = (
        f'[out:json][timeout:60];'
        f'way["highway"="raceway"]["sport"="motor"]({lat_min},{lon_min},{lat_max},{lon_max});'
        f'out geom;'
    )
    data = overpass_query(query)
    segments = []
    for way in data["elements"]:
        if way["type"] != "way" or "geometry" not in way or len(way["geometry"]) < 2:
            continue
        if way["tags"].get("name") == "Pit Lane":
            continue
        pts = [(pt["lat"], pt["lon"]) for pt in way["geometry"]]
        width = way_width(way["tags"])
        if width is None:
            width = FALLBACK_WIDTH_M
        segments.append((pts, width))
    return segments


def chain_segments(segments):
    """Greedy nearest-endpoint chaining into one closed loop -- the relation's member order
    isn't reliably sequential along the route (see module docstring, point 2)."""
    remaining = segments.copy()
    pts0, width0 = remaining.pop(0)
    chain = [(pts0, width0)]
    while remaining:
        tail = chain[-1][0][-1]
        best_idx, best_flip, best_dist = None, False, float("inf")
        for idx, (pts, _) in enumerate(remaining):
            d_fwd = haversine_m(tail, pts[0])
            d_bwd = haversine_m(tail, pts[-1])
            if d_fwd < best_dist:
                best_dist, best_idx, best_flip = d_fwd, idx, False
            if d_bwd < best_dist:
                best_dist, best_idx, best_flip = d_bwd, idx, True
        pts, width = remaining.pop(best_idx)
        if best_flip:
            pts = pts[::-1]
        chain.append((pts, width))

    gaps = [
        haversine_m(chain[i][0][-1], chain[(i + 1) % len(chain)][0][0])
        for i in range(len(chain))
    ]
    return chain, max(gaps)


def despike_offset(
    s, center_offset, half_width,
    jump_threshold=14.0, outlier_threshold=10.0,
    hysteresis_high=20.0, hysteresis_low=10.0,
):
    """The nearest-OSM-point lookup below is a pure 2D spatial search, blind to track
    topology, and fails in two different shapes -- each needs its own detector:

    1. Sharp snap-to-wrong-lobe: a run of samples jumps onto a stable but wrong
       center_offset plateau (tens of meters off), bounded by sharp single-sample jumps
       in and out. Most visibly Suzuka's crossover point (the track physically passes
       over/under itself) and Shanghai's tightly nested corners. Caught below by
       `jump_threshold`/`outlier_threshold`: detect plateaus by their boundary jumps
       (a real corner's smooth offset swing never contains a jump this large), keep only
       the ones whose mean is actually far from the track-wide median -- a magnitude
       threshold alone would miss plateaus that happen to sit just under it (observed on
       Suzuka: one plateau at -16m, next at +15m, straddling any single cutoff).

    2. Gradual multi-way drift: found on Singapore, where the nearest match wanders
       through a sequence of different OSM ways (a nearby paddock/pit-access road,
       probably), climbing smoothly to an offset of 88m over ~180m of track and back --
       no single jump between consecutive samples ever exceeds `jump_threshold`, so
       detector 1 misses most of it. Caught below by hysteresis thresholding (as in
       Canny edge detection): a point beyond `hysteresis_high` seeds a bad region, which
       then floods outward through neighbors while they stay above the looser
       `hysteresis_low` -- this bounds the region at where the drift actually returns to
       near-baseline, rather than at some fixed magnitude.

    Both detectors' bad masks are OR'd together, then linearly interpolated across from
    the surrounding good samples.

    Thresholds sit in the gap between two observed clusters: Baku's genuine narrow
    "castle section" chicane produces real jumps/deviations up to ~13.5m, and Monaco's
    genuine registration noise peaks at ~16.4m deviation from median with no sharp jumps
    at all (checked directly against both tracks' exported CSVs -- tighter thresholds
    previously flagged 30 real Baku samples). Suzuka and Shanghai's wrong-lobe snaps
    start at ~16m and run up to 95m; Singapore's drift runs from ~14m up to 88m. There's
    no guarantee every future track's real geometry stays under these lines, so a large
    despike count on a new track is worth a second look rather than trusting it blindly.
    """
    diffs = np.diff(center_offset)
    boundaries = np.where(np.abs(diffs) > jump_threshold)[0] + 1
    seg_starts = np.concatenate([[0], boundaries])
    seg_ends = np.concatenate([boundaries, [len(center_offset)]])
    median = np.median(center_offset)
    bad = np.zeros(len(center_offset), dtype=bool)
    for a, b in zip(seg_starts, seg_ends):
        if abs(center_offset[a:b].mean() - median) > outlier_threshold:
            bad[a:b] = True

    dev = np.abs(center_offset - median)
    hyst_bad = dev > hysteresis_high
    changed = True
    while changed:
        changed = False
        grow_fwd = np.zeros_like(hyst_bad)
        grow_fwd[1:] = hyst_bad[:-1] & ~hyst_bad[1:] & (dev[1:] > hysteresis_low)
        grow_bwd = np.zeros_like(hyst_bad)
        grow_bwd[:-1] = hyst_bad[1:] & ~hyst_bad[:-1] & (dev[:-1] > hysteresis_low)
        grow = grow_fwd | grow_bwd
        if grow.any():
            hyst_bad |= grow
            changed = True
    bad |= hyst_bad

    n_bad = int(bad.sum())
    if n_bad == 0:
        return center_offset, half_width, 0
    good = ~bad
    period = (s[-1] - s[0]) * len(s) / (len(s) - 1)  # pad by one average sample gap
    center_fixed = center_offset.copy()
    half_width_fixed = half_width.copy()
    center_fixed[bad] = np.interp(s[bad], s[good], center_offset[good], period=period)
    half_width_fixed[bad] = np.interp(s[bad], s[good], half_width[good], period=period)
    return center_fixed, half_width_fixed, n_bad


def project_to_local_meters(lats, lons):
    lat0, lon0 = lats.mean(), lons.mean()
    x = EARTH_RADIUS_M * np.radians(lons - lon0) * np.cos(np.radians(lat0))
    y = EARTH_RADIUS_M * np.radians(lats - lat0)
    return np.column_stack([x, y])


def kabsch(src, dst):
    """Best-fit rotation+translation mapping src -> dst (no reflection)."""
    src_c, dst_c = src.mean(axis=0), dst.mean(axis=0)
    h = (src - src_c).T @ (dst - dst_c)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1, d]) @ u.T
    t = dst_c - r @ src_c
    return r, t


def register(osm_pts, ref_pts, angle_step_deg=2):
    """ICP: brute-force initial rotation (no reflection search -- see module docstring,
    point 3), then iterated closest-point + Kabsch refinement."""
    osm_c = osm_pts - osm_pts.mean(axis=0)
    ref_c = ref_pts - ref_pts.mean(axis=0)
    ref_tree_coarse = cKDTree(ref_c[::3])

    best_angle, best_score = None, np.inf
    for deg in range(0, 360, angle_step_deg):
        th = np.radians(deg)
        r_try = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        cand = (r_try @ osm_c.T).T
        dist, _ = ref_tree_coarse.query(cand[::5])
        score = dist.mean()
        if score < best_score:
            best_score, best_angle = score, deg

    th = np.radians(best_angle)
    r_init = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    current = (r_init @ osm_c.T).T + ref_pts.mean(axis=0)

    ref_tree = cKDTree(ref_pts)
    for _ in range(100):
        _, idx = ref_tree.query(current)
        r, t = kabsch(current, ref_pts[idx])
        new = (r @ current.T).T + t
        shift = np.abs(new - current).max()
        current = new
        if shift < 1e-6:
            break

    dist, _ = ref_tree.query(current)
    return current, dist.mean()


def parse_bbox(s):
    parts = [float(x) for x in s.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("bbox must be 'lat_min,lon_min,lat_max,lon_max'")
    return tuple(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--track", required=True, help="Track slug matching data/<slug>_track_geometry.csv")
    parser.add_argument("--relation-name", help="OSM circuit relation name search string, e.g. 'Marina Bay' (street circuits)")
    parser.add_argument("--bbox", type=parse_bbox, help="'lat_min,lon_min,lat_max,lon_max' fallback for circuits with no relation (permanent circuits)")
    parser.add_argument("--jump-threshold", type=float, default=14.0, help="despike_offset: per-step jump size (m) that seeds a bad plateau (default 14.0, tuned against Baku/Monaco/Suzuka/Shanghai; override per-track if the default's cross-track compromise doesn't separate this track's own genuine vs. artifact magnitudes)")
    parser.add_argument("--outlier-threshold", type=float, default=10.0, help="despike_offset: how far a jump-bounded segment's mean must sit from the track median to count as bad (default 10.0)")
    parser.add_argument("--hysteresis-high", type=float, default=20.0, help="despike_offset: deviation from median (m) that seeds a bad region for the gradual-drift detector (default 20.0)")
    parser.add_argument("--hysteresis-low", type=float, default=10.0, help="despike_offset: deviation from median (m) a bad region floods outward through (default 10.0)")
    args = parser.parse_args()
    slug = args.track.lower()
    if not args.relation_name and not args.bbox:
        parser.error("provide --relation-name and/or --bbox")

    segments = None
    if args.relation_name:
        print(f"Querying OSM for circuit relation matching '{args.relation_name}'...")
        segments = fetch_segments_from_relation(args.relation_name)
    if segments is None:
        if not args.bbox:
            raise ValueError(f"No OSM circuit relation found matching name '{args.relation_name}', and no --bbox given as a fallback.")
        print(f"No relation found; querying highway=raceway ways in bbox {args.bbox}...")
        segments = fetch_segments_from_bbox(args.bbox)
    print(f"{len(segments)} non-pitlane ways")

    # Chaining into one ordered closed loop is only needed for the length sanity-check below,
    # not for registration or width lookup (both operate on point clouds, order-independent).
    # Well-mapped relations chain cleanly (Singapore: 0 m gap); raw bbox-queried ways can be
    # fragmented (Suzuka: many small gaps from granular per-corner tagging) without that
    # meaning the underlying points are wrong -- so a bad chain is a warning, not a hard error.
    chain, max_gap = chain_segments(segments)
    if max_gap > 20.0:
        print(
            f"NOTE: reassembled polyline still has a {max_gap:.1f} m max gap -- source way "
            "geometry is too fragmented to form one clean ordered loop. Proceeding anyway "
            "since registration and width lookup don't need an ordered chain, but the length "
            "sanity-check below is less trustworthy than a clean chain would give."
        )
    else:
        print(f"Chained into a closed loop, max gap {max_gap:.2f} m")

    lats, lons, widths = [], [], []
    for pts, width in segments:
        for lat, lon in pts:
            lats.append(lat)
            lons.append(lon)
            widths.append(width)
    lats, lons, widths = np.array(lats), np.array(lons), np.array(widths)
    osm_pts = project_to_local_meters(lats, lons)

    ref = pd.read_csv(f"data/{slug}_track_geometry.csv")
    ref_pts = ref[["X", "Y"]].values

    total_osm_len = sum(
        haversine_m(pts[j], pts[j + 1]) for pts, _ in chain for j in range(len(pts) - 1)
    )
    print(f"OSM polyline length: {total_osm_len:.0f} m (reference track length for comparison only; unreliable if the chain didn't close)")

    registered, residual = register(osm_pts, ref_pts)
    print(f"Registered OSM road centerline to FastF1 driven line: mean residual {residual:.2f} m")
    if residual > 15.0:
        print(
            f"WARNING: {residual:.2f} m mean residual is larger than the ~5-10 m a genuine "
            "racing-line-vs-road-centerline gap should produce. This may be a bad registration "
            "(wrong rotation, or the relation includes roads not on the actual lap) rather than "
            "real racing-line deviation -- inspect a plot before trusting this output."
        )

    # For each reference-line point, find the nearest point on the registered OSM road
    # centerline and project the offset onto the reference line's own local normal.
    s = ref["Distance"].values
    cx, cy = ref_pts[:, 0], ref_pts[:, 1]
    tx = np.gradient(cx, s)
    ty = np.gradient(cy, s)
    mag = np.hypot(tx, ty)
    tx, ty = tx / mag, ty / mag
    nx, ny = -ty, tx  # left-of-driving-direction normal, matching export_boundaries.py's convention

    osm_tree = cKDTree(registered)
    _, nearest = osm_tree.query(ref_pts)
    dx = registered[nearest, 0] - cx
    dy = registered[nearest, 1] - cy
    center_offset = dx * nx + dy * ny
    half_width = widths[nearest] / 2.0

    center_offset, half_width, n_despiked = despike_offset(
        s, center_offset, half_width,
        jump_threshold=args.jump_threshold, outlier_threshold=args.outlier_threshold,
        hysteresis_high=args.hysteresis_high, hysteresis_low=args.hysteresis_low,
    )
    if n_despiked:
        print(
            f"Despiked {n_despiked} sample(s) where the nearest-OSM-point lookup snapped "
            "onto the wrong lobe of the track (e.g. a crossover or tightly nested corner) -- "
            "replaced with interpolation from surrounding good samples."
        )

    n_left = center_offset + half_width
    n_right = center_offset - half_width

    out = pd.DataFrame({"s": s, "x": cx, "y": cy, "n_left": n_left, "n_right": n_right})
    out_path = f"data/{slug}_track_boundaries.csv"
    out.to_csv(out_path, index=False)

    avg_width = (n_left - n_right).mean()
    print(f"Exported {len(out)} boundary samples -> {out_path}")
    print(f"Average track width: {avg_width:.1f} m (left {n_left.mean():.1f} m, right {abs(n_right.mean()):.1f} m)")


if __name__ == "__main__":
    main()
