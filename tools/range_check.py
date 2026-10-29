# -*- coding: utf-8 -*-
"""
Where does an object at a known distance actually show up?

Compare two captures of the same bench - one with the scene empty, one with
the object placed at a measured distance - and report, for every output layer
the radar has (point cloud, tracks, the FCWS moving / static objects), the
places that are occupied with the object present but not without it.

Subtracting the empty scene is the point: a bench is full of walls, tables
and multipath ghosts, and a detection at 8-11 m means nothing until it is
shown to come and go with the object.

Usage:
    python tools/range_check.py empty/objects_*.csv object_1m/objects_*.csv --truth 1.0
    python tools/range_check.py empty.csv obj.csv --truth 2.0 --lateral 0.3

The objects_*.csv files are the long-format logs the processor writes
(distX longitudinal, distY lateral, one row per object per frame).
"""

import argparse
import collections
import csv
import math

CELL_M = 0.5            # occupancy grid resolution, both axes
MIN_EXTRA_RATE = 0.20   # appears in at least this many more frames per frame
NEAR_TRUTH_M = 1.0      # "found where it is" tolerance

LAYERS = (("point", "point cloud"), ("track", "tracks"),
          ("dynamic", "FCWS moving object (0x510)"),
          ("static", "FCWS static object (0x590)"))


def load(path):
    """{layer: [(frame, long, lat)]} and the number of frames."""
    rows = collections.defaultdict(list)
    frames = set()
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            frames.add(row["frame_num"])
            try:
                long_m, lat_m = float(row["distX"]), float(row["distY"])
            except (TypeError, ValueError):
                continue
            rows[row["source"]].append((row["frame_num"], long_m, lat_m))
    return rows, max(len(frames), 1)


def occupancy(detections, n_frames):
    """Fraction of frames in which each cell held at least one detection."""
    seen = collections.defaultdict(set)
    for frame, long_m, lat_m in detections:
        seen[(math.floor(long_m / CELL_M), math.floor(lat_m / CELL_M))].add(frame)
    return {cell: len(fr) / n_frames for cell, fr in seen.items()}


def compare(empty_path, object_path, truth_m, truth_lat=0.0):
    empty, n_empty = load(empty_path)
    obj, n_obj = load(object_path)
    report = {}
    for layer, _ in LAYERS:
        before = occupancy(empty.get(layer, []), n_empty)
        after = occupancy(obj.get(layer, []), n_obj)
        extra = sorted(((rate - before.get(cell, 0.0), cell, rate)
                        for cell, rate in after.items()), reverse=True)
        new_cells = [{
            "range_m": (cell[0] + 0.5) * CELL_M,
            "lateral_m": (cell[1] + 0.5) * CELL_M,
            "extra_rate": round(gain, 2),
            "rate_with_object": round(rate, 2),
        } for gain, cell, rate in extra if gain >= MIN_EXTRA_RATE]

        near_frames = {frame for frame, long_m, lat_m in obj.get(layer, [])
                       if math.hypot(long_m - truth_m, lat_m - truth_lat) <= NEAR_TRUTH_M}
        report[layer] = {
            "new_cells": new_cells[:6],
            "found_at_truth_rate": round(len(near_frames) / n_obj, 2),
        }
    return report, n_empty, n_obj


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("empty", help="objects_*.csv with the scene empty")
    parser.add_argument("with_object", help="objects_*.csv with the object placed")
    parser.add_argument("--truth", type=float, required=True,
                        help="measured distance to the object along boresight, m")
    parser.add_argument("--lateral", type=float, default=0.0,
                        help="measured lateral offset, m (right positive)")
    args = parser.parse_args(argv)

    report, n_empty, n_obj = compare(args.empty, args.with_object,
                                     args.truth, args.lateral)
    print(f"Object placed at {args.truth:.2f} m, lateral {args.lateral:+.2f} m"
          f"   ({n_empty} empty frames, {n_obj} frames with object)\n")
    for layer, title in LAYERS:
        entry = report[layer]
        print(f"{title}:")
        print(f"  detected within {NEAR_TRUTH_M:.0f} m of the true position in "
              f"{entry['found_at_truth_rate']:.0%} of frames")
        if not entry["new_cells"]:
            print("  nothing new appears with the object present")
        for cell in entry["new_cells"]:
            print(f"  new at {cell['range_m']:6.2f} m, lateral {cell['lateral_m']:+5.2f} m"
                  f"   in {cell['rate_with_object']:.0%} of frames"
                  f" (+{cell['extra_rate']:.0%} vs empty)")
        print()


if __name__ == "__main__":
    main()
