"""Solver regression check: invariants that must hold, plus a diff against a saved run.

Complements validate_pole_times.py, which measures accuracy against reality (and needs the
network). This one is offline and asks a different question: is the solver still internally
consistent, and did a change move any lap time I didn't expect it to?

Three checks, in order of how much they've actually caught:

1. **Pinned-width regression.** Shrink a track's boundaries until the car has no room to
   move, and the racing-line solver must reproduce the fixed-line solver exactly -- n=0 is
   a feasible point of the same problem. Any difference is the solver finding time that
   isn't there. This is the sharpest tool here: it caught the odd-even checkerboard
   artifact that a whole release's regularization had only been pricing rather than
   removing, and it's how the residual sub-grid artifact was found (see DISCUSSION.md V7).
   Run at two widths. Zero usable width must give exactly 0.000 -- anything else is a
   straight bug. One centimetre is the sensitive one: it *should* buy nothing, but
   currently buys 0.3-1.1 s because a sub-grid wiggle's curvature scales as n/ds^2 (V7
   again). That is a known, documented limitation, so it is reported rather than failed --
   what fails is it getting *worse* than a saved baseline.

2. **Every working track converges**, with the racing line never slower than the fixed
   line. A non-converged solve used to be returned as a result; now it errors, and this
   catches it across the whole set rather than one track at a time.

3. **Diff against a saved run.** --save records lap times; --compare reports what moved.
   A refactor should move nothing; a physics change should move what you predicted and
   nothing else.

Usage:
  python python_scripts/check_solver.py --save baseline.json
  ... make a change, rebuild ...
  python python_scripts/check_solver.py --compare baseline.json

Requires a release build (cargo build --release) and the usual per-track data/ files.
"""

import argparse
import csv
import json
import re
import shutil
import statistics
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
BINARY = REPO / "target/release/f1-lap-sim"
DATA = REPO / "data"

# Previously this skipped Bahrain, Catalunya and Singapore, whose OpenStreetMap boundaries
# were unusable. With measured widths from the TUM racetrack-database every circuit solves,
# so nothing is skipped; keep the hook for the next track that turns out to be broken.
EXCLUDED: set[str] = set()

# Boundary half-widths for the pinned runs. track.rs subtracts CAR_HALF_WIDTH_M (1.0 m)
# from each side, so 1.0 leaves the car exactly no room and 1.01 leaves it a centimetre.
# If CAR_HALF_WIDTH_M ever changes, these have to change with it.
PIN_ZERO = 1.0
PIN_1CM = 1.01
PIN_PREFIX = "zzcheck"


def working_tracks():
    slugs = sorted(
        p.name[: -len("_track_boundaries.csv")]
        for p in DATA.glob("*_track_boundaries.csv")
        if not p.name.startswith(PIN_PREFIX)
    )
    return [s for s in slugs if s not in EXCLUDED]


def solve(slug, spacing):
    out = subprocess.run(
        [str(BINARY), slug, "racingline", str(spacing)],
        cwd=REPO, capture_output=True, text=True,
    )
    text = out.stdout + out.stderr
    racing = re.search(r"Lap time \(racing line\): (\d+):([\d.]+)", text)
    fixed = re.search(r"Fixed-line optimal lap time for comparison: ([\d.]+)s", text)
    return {
        "ok": out.returncode == 0,
        "statuses": re.findall(r"Ipopt status: (\w+)", text),
        "racing": int(racing.group(1)) * 60 + float(racing.group(2)) if racing else None,
        "fixed": float(fixed.group(1)) if fixed else None,
        "error": None if out.returncode == 0 else text.strip().splitlines()[-1][:160],
    }


def pin_track(slug, half_width):
    """Copy a track's data with its boundaries pinned to a fixed half-width."""
    pinned = f"{PIN_PREFIX}{slug}"
    for suffix in ("track_geometry", "drs_zones", "aero_params", "reference_lap"):
        src = DATA / f"{slug}_{suffix}.csv"
        if src.exists():
            shutil.copy(src, DATA / f"{pinned}_{suffix}.csv")
    with open(DATA / f"{slug}_track_boundaries.csv") as src:
        rows = list(csv.DictReader(src))
    with open(DATA / f"{pinned}_track_boundaries.csv", "w", newline="") as dst:
        writer = csv.DictWriter(dst, fieldnames=rows[0].keys())
        writer.writeheader()
        for row in rows:
            row["n_left"], row["n_right"] = str(half_width), str(-half_width)
            writer.writerow(row)
    return pinned


def remove_pinned(pinned):
    for path in DATA.glob(f"{pinned}_*"):
        path.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--spacing", type=float, default=5.0, help="collocation spacing (default: the CLI's own default)")
    parser.add_argument("--tracks", nargs="+", help="check only these slugs")
    parser.add_argument("--pinned-tracks", nargs="+", default=["monaco", "monza", "suzuka"],
                        help="tracks used for the pinned-width regression (default: a slow, a fast and a mixed circuit)")
    parser.add_argument("--save", help="write results to this JSON file")
    parser.add_argument("--compare", help="diff against a previously saved JSON file")
    args = parser.parse_args()

    if not BINARY.exists():
        raise SystemExit(f"{BINARY} not found -- run `cargo build --release` first")

    tracks = args.tracks or working_tracks()
    previous = json.load(open(args.compare)) if args.compare else None
    results = {"spacing": args.spacing, "tracks": {}, "pinned": {}}
    problems = []

    header = f"{'track':<13}{'racing':>10}{'fixed':>10}{'racing-fixed':>14}  status"
    print(header + ("   vs saved" if previous else ""))
    for slug in tracks:
        result = solve(slug, args.spacing)
        results["tracks"][slug] = result
        if result["racing"] is None:
            problems.append(f"{slug}: {result['error']}")
            print(f"{slug:<13}{'FAILED':>10}  {result['error']}")
            continue
        delta = result["racing"] - result["fixed"]
        line = f"{slug:<13}{result['racing']:>10.3f}{result['fixed']:>10.3f}{delta:>+14.3f}  {','.join(result['statuses'])}"
        if previous and (before := previous["tracks"].get(slug, {}).get("racing")):
            line += f"   {result['racing'] - before:>+8.3f}"
        print(line)
        if any(s != "SolveSucceeded" for s in result["statuses"]):
            problems.append(f"{slug}: Ipopt reported {result['statuses']}")
        if delta > 0.05:
            problems.append(f"{slug}: racing line slower than fixed line by {delta:.3f}s")

    print("\npinned-width regression (no room to move -> the two solvers must agree;")
    print("the 1 cm rows are the known sub-grid residual, see DISCUSSION.md V7):")
    # Zero width is an absolute requirement; 1 cm is a known residual, judged against the
    # saved baseline instead of an absolute tolerance it currently cannot meet.
    for label, half_width, tolerance in (("zero width", PIN_ZERO, 0.001), ("1 cm", PIN_1CM, None)):
        results["pinned"][label] = {}
        for slug in args.pinned_tracks:
            pinned = pin_track(slug, half_width)
            try:
                result = solve(pinned, args.spacing)
            finally:
                remove_pinned(pinned)
            results["pinned"][label][slug] = result
            if result["racing"] is None:
                problems.append(f"pinned {label} {slug}: {result['error']}")
                print(f"  {label:<11}{slug:<12} FAILED {result['error']}")
                continue
            gap = result["racing"] - result["fixed"]
            note = ""
            if previous and (before := previous.get("pinned", {}).get(label, {}).get(slug)):
                note = f"   (was {before['racing'] - before['fixed']:+.3f})"
            print(f"  {label:<11}{slug:<12} racing {result['racing']:8.3f}  fixed {result['fixed']:8.3f}  gap {gap:+.3f}s{note}")
            if tolerance is not None and abs(gap) > tolerance:
                problems.append(
                    f"pinned {label} {slug}: solvers disagree by {abs(gap):.3f}s "
                    f"(tolerance {tolerance}s) -- the racing line is finding time that isn't there"
                )
            elif tolerance is None and previous:
                before_gap = previous.get("pinned", {}).get(label, {}).get(slug)
                if before_gap and before_gap["racing"] is not None:
                    grew = abs(gap) - abs(before_gap["racing"] - before_gap["fixed"])
                    if grew > 0.1:
                        problems.append(
                            f"pinned {label} {slug}: the known sub-grid artifact grew by "
                            f"{grew:.3f}s (now {abs(gap):.3f}s)"
                        )

    solved = [r for r in results["tracks"].values() if r["racing"] is not None]
    if solved:
        gains = [r["racing"] - r["fixed"] for r in solved]
        print(f"\n{len(solved)}/{len(tracks)} tracks solved; racing line gains "
              f"{min(gains):.2f}..{max(gains):.2f}s (mean {statistics.mean(gains):.2f}s)")

    if args.save:
        json.dump(results, open(args.save, "w"), indent=1)
        print(f"saved to {args.save}")

    print("\nPROBLEMS:" if problems else "\nno problems flagged")
    for problem in problems:
        print(f"  - {problem}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
