"""
Compare the solver's lap times against real 2018 qualifying pole times, across all 21 rounds
of the season.

Reports both modes, because they mean different things (see results/DISCUSSION.md V7):

  fixed line   -- the car is pinned to the lap the driver actually drove and only the speed
                  profile is optimized. This is the number that validates the physics, and
                  the one to watch when changing the car model or the aero derivation.
  racing line  -- the solver also picks the path within the track width. It still reads
                  faster than most real laps, so it is reported for context rather than as a
                  prediction. The suspected causes are the sub-grid weave and the point-mass
                  model's cheap lateral freedom, not the corridor.

This check drove several fixes: the original 3-tier downforce classification showed a
systematic +16.3% mean gap (worse the more corner-heavy the track), which DRS modeling and
per-track downforce derivation brought down sharply. Committed as a re-runnable script
rather than the one-off comparisons used to develop those fixes.

The benchmark is the pole lap: session.laps.pick_fastest(), the outright fastest Q lap by
any driver. That is the question this project asks -- how close does the solver get to the
quickest lap anyone actually set at each 2018 round -- so it is the right target even though
the geometry currently comes from Hamilton's lap. Where those differ (12 of the 21 rounds,
by up to 1.8 s) the solve is pinned to one driver's line and scored against another's time;
see results/DISCUSSION.md for why that is a geometry problem to fix at the source, not a
benchmark to change.

Fetched live via FastF1 rather than hardcoded, so it stays correct if FastF1's own data is
ever corrected upstream. Cached locally after the first run like every other script here.

src/api.rs keeps a static copy of the same figures (POLE_TIME_2018) so the web UI can show
them without a network call. This script is the authority; if the two disagree, that table
is the stale one.

Every run appends one row per track to results/validation_log.csv, so a later run can be
compared against earlier ones per track instead of only by the summary means.

Usage:
  python validate_pole_times.py                  # all 21 tracks
  python validate_pole_times.py --track monaco   # just one
  python validate_pole_times.py --no-log         # don't record the run

Requires a built release binary (cargo build --release) and each track's
data/<slug>_track_geometry.csv / data/<slug>_track_boundaries.csv to already exist.
"""

import argparse
import csv
import datetime
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
# Append-only history of every run, committed to the repo (unlike data/, results/ is tracked).
# Without it each run's numbers exist only in a terminal that later scrolls away, so there is
# no way to tell afterwards whether a model change moved each track the right way or merely
# improved the mean -- which is how README/DISCUSSION ended up quoting superseded figures.
LOG_PATH = Path("results/validation_log.csv")
LOG_COLUMNS = [
    "run_utc", "commit", "dirty", "spacing_m", "track",
    "fixed_line_s", "racing_line_s", "real_pole_s", "fixed_gap_pct", "racing_gap_pct",
]
SPACING_M = 5.0
LAP_TIME_RE = re.compile(r"Lap time \(racing line\): (\d+):(\d+\.\d+)")
FIXED_LINE_RE = re.compile(r"Fixed-line optimal lap time for comparison: (\d+\.\d+)s")


def real_pole_seconds(event_name: str) -> float:
    session = fastf1.get_session(2018, event_name, "Q")
    session.load(telemetry=False, weather=False, messages=False)
    if session.event["EventName"] != event_name:
        raise ValueError(f"{event_name!r} resolved to {session.event['EventName']!r} -- check the name")
    return session.laps.pick_fastest()["LapTime"].total_seconds()


def solver_seconds(slug: str) -> tuple[float, float]:
    """Both lap times from one run: racingline mode prints the fixed-line time too."""
    result = subprocess.run(
        [str(BINARY), slug, "racingline", str(SPACING_M)],
        capture_output=True, text=True, check=True,
    )
    racing = LAP_TIME_RE.search(result.stdout)
    fixed = FIXED_LINE_RE.search(result.stdout)
    if not racing or not fixed:
        raise ValueError(f"{slug}: couldn't find both lap times in the solver output")
    minutes, seconds = racing.groups()
    return int(minutes) * 60 + float(seconds), float(fixed.group(1))


def git_state() -> tuple[str, bool]:
    """Short HEAD and whether the tree was dirty. A dirty tree means the commit does not
    fully identify what produced the row, so it is recorded rather than silently ignored."""
    def git(*args):
        return subprocess.run(["git", *args], capture_output=True, text=True, check=True).stdout.strip()
    try:
        # Exclude the log itself: appending to it dirties the tree, so counting it would
        # pin the flag to 1 from the second run onwards and tell us nothing.
        changed = [line for line in git("status", "--porcelain").splitlines()
                   if line[3:].strip() != str(LOG_PATH)]
        return git("rev-parse", "--short", "HEAD"), bool(changed)
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown", True


def append_log(rows, run_utc: str, commit: str, dirty: bool) -> None:
    """One row per track. Rows sharing a run_utc are one run, so a --track run is
    distinguishable from a full sweep by counting them."""
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    is_new = not LOG_PATH.exists()
    # newline="" is csv's required open mode; lineterminator pins LF so this tracked file
    # does not land CRLF in the repo on a platform whose default differs.
    with LOG_PATH.open("a", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        if is_new:
            writer.writerow(LOG_COLUMNS)
        for slug, fixed, racing, real, fixed_pct, racing_pct in rows:
            writer.writerow([
                run_utc, commit, int(dirty), SPACING_M, slug,
                f"{fixed:.3f}", f"{racing:.3f}", f"{real:.3f}",
                f"{fixed_pct:.2f}", f"{racing_pct:.2f}",
            ])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--track", choices=sorted(TRACKS), help="Check just one track (default: all)")
    parser.add_argument("--no-log", action="store_true",
                        help=f"Print the table without appending to {LOG_PATH}")
    args = parser.parse_args()

    if not BINARY.exists():
        raise SystemExit(f"{BINARY} not found -- run `cargo build --release` first")

    run_utc = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")
    tracks = {args.track: TRACKS[args.track]} if args.track else TRACKS
    rows = []
    for slug, event_name in tracks.items():
        real = real_pole_seconds(event_name)
        racing, fixed = solver_seconds(slug)
        rows.append((slug, fixed, racing, real,
                     (fixed - real) / real * 100, (racing - real) / real * 100))

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

    if args.no_log:
        return
    commit, dirty = git_state()
    append_log(rows, run_utc, commit, dirty)
    print(f"\nAppended {len(rows)} row(s) to {LOG_PATH} (commit {commit}"
          + (", working tree dirty)" if dirty else ")"))


if __name__ == "__main__":
    main()
