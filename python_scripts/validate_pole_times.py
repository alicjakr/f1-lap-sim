"""
Compare the solver's lap times against real 2018 qualifying pole times, across every track
with working OSM boundary data.

Reports both modes, because they mean different things (see results/DISCUSSION.md V7):

  fixed line   -- the car is pinned to the lap the driver actually drove and only the speed
                  profile is optimized. This is the number that validates the physics, and
                  the one to watch when changing the car model or the aero derivation.
  racing line  -- the solver also picks the path within the track width. It currently reads
                  faster than any real lap, because the corridor is centred on a reference
                  lap that was already a racing line, so the line optimization spends width
                  the driver had already used. Reported for context, not as a prediction.

This check drove several fixes: the original 3-tier downforce classification showed a
systematic +16.3% mean gap (worse the more corner-heavy the track), which DRS modeling and
per-track downforce derivation brought down sharply. Committed as a re-runnable script
rather than the one-off comparisons used to develop those fixes.

Real pole times are fetched live via FastF1 (session.laps.pick_fastest(), i.e. the outright
fastest lap in Q by any driver -- not necessarily the same driver/lap used to export each
track's geometry) rather than hardcoded, so this stays correct if FastF1's own data is ever
corrected upstream. Cached locally after the first run like every other script here.

src/api.rs keeps a static copy of the same figures (POLE_TIME_2018) so the web UI can show
them without a network call. This script is the authority; if the two disagree, that table
is the stale one.

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

# slug -> official 2018 FastF1 EventName. All 21 rounds of the season now run end to end:
# the six that used to fail did so for want of usable boundaries, which the TUM
# racetrack-database supplies (see python_scripts/export_tum_boundaries.py).
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
    "bahrain": "Bahrain Grand Prix",
    "catalunya": "Spanish Grand Prix",
    "melbourne": "Australian Grand Prix",
    "mexico": "Mexican Grand Prix",
    "singapore": "Singapore Grand Prix",
    "sochi": "Russian Grand Prix",
}

BINARY = Path("target/release/f1-lap-sim")
LAP_TIME_RE = re.compile(r"Lap time \(racing line\): (\d+):(\d+\.\d+)")
FIXED_LINE_RE = re.compile(r"Fixed-line optimal lap time for comparison: (\d+\.\d+)s")


def real_pole_seconds(event_name: str) -> float:
    session = fastf1.get_session(2018, event_name, "Q")
    session.load(telemetry=False, weather=False, messages=False)
    if session.event["EventName"] != event_name:
        raise ValueError(f"{event_name!r} resolved to {session.event['EventName']!r} -- check the name")
    fastest = session.laps.pick_fastest()
    return fastest["LapTime"].total_seconds()


def solver_seconds(slug: str) -> tuple[float, float]:
    """Both lap times from one run: racingline mode prints the fixed-line time too."""
    result = subprocess.run(
        [str(BINARY), slug, "racingline", "5"],
        capture_output=True, text=True, check=True,
    )
    racing = LAP_TIME_RE.search(result.stdout)
    fixed = FIXED_LINE_RE.search(result.stdout)
    if not racing or not fixed:
        raise ValueError(f"{slug}: couldn't find both lap times in the solver output")
    minutes, seconds = racing.groups()
    return int(minutes) * 60 + float(seconds), float(fixed.group(1))


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
        racing, fixed = solver_seconds(slug)
        rows.append((slug, fixed, racing, real, (fixed - real) / real * 100, (racing - real) / real * 100))

    rows.sort(key=lambda r: r[4])
    print(f"\n{'track':<13}{'fixed line':>11}{'racing line':>12}{'real pole':>11}{'fixed gap':>11}{'racing gap':>12}")
    for slug, fixed, racing, real, fixed_pct, racing_pct in rows:
        print(f"{slug:<13}{fixed:>11.3f}{racing:>12.3f}{real:>11.3f}{fixed_pct:>+10.1f}%{racing_pct:>+11.1f}%")

    if len(rows) > 1:
        fixed_pcts = [r[4] for r in rows]
        racing_pcts = [r[5] for r in rows]
        print(f"\nfixed line (validated): mean {statistics.mean(fixed_pcts):+.1f}%  "
              f"stdev {statistics.stdev(fixed_pcts):.1f}%")
        print(f"racing line (upper bound, not a prediction): mean {statistics.mean(racing_pcts):+.1f}%  "
              f"stdev {statistics.stdev(racing_pcts):.1f}%")


if __name__ == "__main__":
    main()
