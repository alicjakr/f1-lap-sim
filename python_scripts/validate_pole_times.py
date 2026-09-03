"""
Compare the racing-line solver's lap times against real 2018 qualifying pole times, across
every track with working OSM boundary data.

This is the check that drove several fixes this project went through: the original 3-tier
downforce classification showed a systematic +16.3% mean gap (worse the more corner-heavy
the track), which DRS modeling and per-track downforce derivation (see
derive_downforce.py's docstring for the methodology and validation history) brought down to
+0.9%. Committed here as a real, re-runnable script instead of the one-off scratchpad
comparisons used to develop those fixes, so a future change to the solver, car parameters,
or aero derivation can be checked against the same baseline.

Real pole times are fetched live via FastF1 (session.laps.pick_fastest(), i.e. the outright
fastest lap in Q by any driver -- not necessarily the same driver/lap used to export each
track's geometry) rather than hardcoded, so this stays correct if FastF1's own data is ever
corrected upstream. Cached locally after the first run like every other script here.

Usage:
  python validate_pole_times.py                  # all 15 working tracks
  python validate_pole_times.py --track monaco    # just one

Requires a built release binary (cargo build --release) and each track's
data/<slug>_track_geometry.csv / data/<slug>_track_boundaries.csv to already exist.
"""

import argparse
import re
import statistics
import subprocess
from pathlib import Path

import fastf1

fastf1.Cache.enable_cache("cache")

# slug -> official 2018 FastF1 EventName. Restricted to the 15 tracks with working OSM
# boundary data (see python_scripts/export_osm_boundaries.py's module docstring for the
# other 6: Bahrain/Catalunya have unusable OSM relations, Sochi/Mexico have none at all,
# Singapore hits a real steering-rate model limit unrelated to data quality).
TRACKS = {
    "monaco": "Monaco Grand Prix",
    "baku": "Azerbaijan Grand Prix",
    "suzuka": "Japanese Grand Prix",
    "shanghai": "Chinese Grand Prix",
    "monza": "Italian Grand Prix",
    "spa": "Belgian Grand Prix",
    "montreal": "Canadian Grand Prix",
    "paulricard": "French Grand Prix",
    "silverstone": "British Grand Prix",
    "hockenheim": "German Grand Prix",
    "hungaroring": "Hungarian Grand Prix",
    "cota": "United States Grand Prix",
    "interlagos": "Brazilian Grand Prix",
    "redbullring": "Austrian Grand Prix",
    "yasmarina": "Abu Dhabi Grand Prix",
}

BINARY = Path("target/release/f1-lap-sim")
LAP_TIME_RE = re.compile(r"Lap time \(racing line\): (\d+):(\d+\.\d+)")


def real_pole_seconds(event_name: str) -> float:
    session = fastf1.get_session(2018, event_name, "Q")
    session.load(telemetry=False, weather=False, messages=False)
    if session.event["EventName"] != event_name:
        raise ValueError(f"{event_name!r} resolved to {session.event['EventName']!r} -- check the name")
    fastest = session.laps.pick_fastest()
    return fastest["LapTime"].total_seconds()


def solver_seconds(slug: str) -> float:
    result = subprocess.run(
        [str(BINARY), slug, "racingline", "25"],
        capture_output=True, text=True, check=True,
    )
    match = LAP_TIME_RE.search(result.stdout)
    if not match:
        raise ValueError(f"{slug}: couldn't find \"Lap time (racing line)\" in solver output")
    minutes, seconds = match.groups()
    return int(minutes) * 60 + float(seconds)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--track", choices=sorted(TRACKS), help="Check just one track (default: all)")
    args = parser.parse_args()

    if not BINARY.exists():
        raise SystemExit(f"{BINARY} not found -- run `cargo build --release` first")

    tracks = {args.track: TRACKS[args.track]} if args.track else TRACKS
    rows = []
    for slug, event_name in tracks.items():
        real = real_pole_seconds(event_name)
        sim = solver_seconds(slug)
        pct = (sim - real) / real * 100
        rows.append((slug, sim, real, pct))

    rows.sort(key=lambda r: r[3])
    print(f"\n{'track':<13}{'solver':>10}{'real pole':>11}{'gap':>9}")
    for slug, sim, real, pct in rows:
        print(f"{slug:<13}{sim:>10.3f}{real:>11.3f}{pct:>+8.1f}%")

    if len(rows) > 1:
        pcts = [r[3] for r in rows]
        print(f"\nmean gap: {statistics.mean(pcts):+.1f}%  stdev: {statistics.stdev(pcts):.1f}%")


if __name__ == "__main__":
    main()
