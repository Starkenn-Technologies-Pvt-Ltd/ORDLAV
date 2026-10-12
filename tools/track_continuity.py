# -*- coding: utf-8 -*-
"""
Report object identity continuity from an objects_*.csv log.

Answers the questions function-performance review actually asks of a run:
does one physical target stay one object, where does it split, where does it
drop out, and how often does its identity churn.

Important: the identities are assigned by object_tracker during capture, not
reported by the radar - no documented message carries an object ID. So this
measures the radar's detection stability, which is what a split or a dropout
really is. It cannot audit the sensor's internal ID handling, because the
sensor exposes none.

Usage:
    python tools/track_continuity.py objects_12_04_10_12_2026.csv
    python tools/track_continuity.py objects_*.csv --source track --split-gap 4
"""

import argparse
import csv
import glob
import math
from collections import defaultdict


def load(path, source):
    """Rows for one source, grouped by frame and by assigned id."""
    by_frame = defaultdict(list)
    by_id = defaultdict(list)
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            if row.get("source") != source:
                continue
            try:
                frame = int(row["frame_num"])
                record = {
                    "frame": frame,
                    "id": int(float(row["assoc_id"])) if row.get("assoc_id") else None,
                    "distX": float(row["distX"]),
                    "distY": float(row["distY"]),
                }
            except (TypeError, ValueError):
                continue
            by_frame[frame].append(record)
            if record["id"] is not None:
                by_id[record["id"]].append(record)
    return by_frame, by_id


def gaps_in(frames):
    """Runs of missing frames between the first and last sighting."""
    present = sorted(set(frames))
    out = []
    for previous, current in zip(present, present[1:]):
        if current - previous > 1:
            out.append((previous + 1, current - 1))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+", help="objects_*.csv file(s)")
    parser.add_argument("--source", default="track",
                        help="Which source column to analyse (default: track)")
    parser.add_argument("--split-gap", type=float, default=4.0, metavar="M",
                        help="Two simultaneous objects closer than this are "
                             "treated as one target split in two (default: 4 m)")
    parser.add_argument("--brief-frames", type=int, default=5,
                        help="Identities lasting fewer frames than this count "
                             "as churn (default: 5)")
    args = parser.parse_args()

    paths = [p for pattern in args.logs for p in sorted(glob.glob(pattern))]
    if not paths:
        raise SystemExit("No matching logs")

    for path in paths:
        by_frame, by_id = load(path, args.source)
        if not by_frame:
            print(f"\n{path}: no '{args.source}' rows")
            continue

        total_frames = len(by_frame)
        frames_present = sorted(by_frame)

        print("\n" + "=" * 70)
        print(f"{path}   source='{args.source}'")
        print("=" * 70)
        print(f"  frames with objects : {total_frames}")
        print(f"  identities assigned : {len(by_id)}")

        print(f"\n  {'id':>4}  {'first':>6}  {'last':>6}  {'seen':>5}  "
              f"{'span':>5}  {'gaps':>5}  range covered")
        churn = []
        for track_id in sorted(by_id):
            records = sorted(by_id[track_id], key=lambda r: r["frame"])
            frames = [r["frame"] for r in records]
            span = frames[-1] - frames[0] + 1
            holes = gaps_in(frames)
            dist = [r["distX"] for r in records]
            print(f"  {track_id:>4}  {frames[0]:>6}  {frames[-1]:>6}  "
                  f"{len(frames):>5}  {span:>5}  {len(holes):>5}  "
                  f"{max(dist):6.1f} -> {min(dist):5.1f} m")
            if len(frames) < args.brief_frames:
                churn.append(track_id)
            for start, end in holes:
                print(f"        dropout: frames {start}-{end} "
                      f"({end - start + 1} frame(s))")

        # Two identities sitting on top of each other is one target split in two
        splits = []
        for frame in frames_present:
            objects = [o for o in by_frame[frame] if o["id"] is not None]
            for i in range(len(objects)):
                for j in range(i + 1, len(objects)):
                    separation = math.hypot(
                        objects[i]["distX"] - objects[j]["distX"],
                        objects[i]["distY"] - objects[j]["distY"])
                    if separation < args.split_gap:
                        splits.append((frame, objects[i]["id"],
                                       objects[j]["id"], separation))

        print("\n  Findings")
        if splits:
            pairs = defaultdict(list)
            for frame, a, b, sep in splits:
                pairs[(a, b)].append((frame, sep))
            for (a, b), hits in sorted(pairs.items()):
                frames = [f for f, _ in hits]
                closest = min(s for _, s in hits)
                print(f"    SPLIT  ids {a} and {b} coexist within "
                      f"{args.split_gap:g} m on {len(hits)} frame(s) "
                      f"({frames[0]}-{frames[-1]}, closest {closest:.2f} m)")
        else:
            print(f"    No two identities came within {args.split_gap:g} m "
                  f"of each other.")

        if churn:
            print(f"    CHURN  {len(churn)} identity(ies) lasted under "
                  f"{args.brief_frames} frames: {churn}")
        else:
            print(f"    No identity lasted under {args.brief_frames} frames.")

        total_gaps = sum(len(gaps_in([r['frame'] for r in by_id[i]])) for i in by_id)
        if not total_gaps:
            print("    No dropouts within any identity's lifetime.")


if __name__ == "__main__":
    main()
