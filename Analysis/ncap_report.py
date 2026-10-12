# -*- coding: utf-8 -*-
"""
Kinematics validation of radar output against ground truth, per test run.

Produces, for one logged run, the comparison the Euro NCAP AEB protocols'
methodology implies: each kinematic channel plotted against ground truth over
time, the error alongside it, and the detection behaviour summarised.

On pass/fail, read this before trusting a verdict
-------------------------------------------------
The Euro NCAP AEB protocols (Car-to-Car v4.3.1, VRU v4.5.1) specify
*system-level* performance - whether the vehicle avoids the collision or by
how much it reduces speed. They do not define sensor-level accuracy limits,
so they cannot supply the tolerance corridor this report draws. That corridor
has to come from your own radar requirement specification.

Until one exists, run without --tolerances: the report then states the error
it measured and makes no judgement. It also prints observed error percentiles,
which are the right input for *writing* a requirement - but a requirement
derived from one run and then used to grade that same run proves nothing.

What the protocols do usefully give you is the scenario set. Tag each run with
--scenario so results are comparable across the matrix.

Usage:
    python Analysis/ncap_report.py objects_12_04_10_12_2026.csv
    python Analysis/ncap_report.py objects_*.csv --scenario CCRs --source track
    python Analysis/ncap_report.py objects_*.csv --tolerances Analysis/tolerances.json
"""

import argparse
import glob
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

# Scenario labels from the Euro NCAP AEB protocols. Names only - consult the
# protocol for each case's speeds, overlaps and approach geometry.
SCENARIOS = {
    "CCRs": "Car-to-Car Rear stationary",
    "CCRm": "Car-to-Car Rear moving",
    "CCRb": "Car-to-Car Rear braking",
    "CPNA": "Car-to-Pedestrian Nearside Adult",
    "CPNC": "Car-to-Pedestrian Nearside Child",
    "CPFA": "Car-to-Pedestrian Farside Adult",
    "CPLA": "Car-to-Pedestrian Longitudinal Adult",
    "CBNA": "Car-to-Bicyclist Nearside Adult",
    "CBLA": "Car-to-Bicyclist Longitudinal Adult",
}

CHANNELS = [
    ("distX", "Longitudinal distance", "m"),
    ("distY", "Lateral distance", "m"),
    ("vrelX", "Longitudinal relative speed", "m/s"),
    ("vrelY", "Lateral relative speed", "m/s"),
]


def pick_track(frame):
    """The identity present in the most frames; the primary target."""
    counts = frame.groupby("assoc_id").size().sort_values(ascending=False)
    return counts.index[0] if len(counts) else None


def align(objects, radar_source, truth_source, track_id=None):
    """One row per frame with radar and ground-truth columns side by side."""
    truth = objects[objects.source == truth_source]
    radar = objects[objects.source == radar_source]
    if radar_source == "track":
        chosen = track_id if track_id is not None else pick_track(radar)
        if chosen is None:
            return pd.DataFrame(), None
        radar = radar[radar.assoc_id == chosen]
    else:
        chosen = None

    columns = ["frame_num", "timestamp"] + [c for c, _, _ in CHANNELS]
    merged = truth[columns].merge(
        radar[columns], on="frame_num", suffixes=("_gt", "_radar"))
    return merged, chosen


def detection_metrics(merged, truth_frames):
    """How reliably the target was seen, independent of accuracy."""
    detected = set(merged.frame_num)
    expected = sorted(truth_frames)
    missing = [f for f in expected if f not in detected]

    longest, run = 0, 0
    for frame in expected:
        run = run + 1 if frame in missing else 0
        longest = max(longest, run)

    first_range = ""
    if len(merged):
        first = merged.sort_values("frame_num").iloc[0]
        first_range = first["distX_gt"]

    return {
        "frames_with_truth": len(expected),
        "frames_detected": len(detected),
        "detection_rate": 100.0 * len(detected) / len(expected) if expected else 0.0,
        "missed_frames": len(missing),
        "longest_dropout": longest,
        "first_detection_range_m": first_range,
    }


def channel_stats(merged, channel):
    error = (merged[f"{channel}_radar"] - merged[f"{channel}_gt"]).dropna()
    if error.empty:
        return None
    return {
        "n": len(error),
        "mean_error": error.mean(),
        "mae": error.abs().mean(),
        "rmse": (error ** 2).mean() ** 0.5,
        "std": error.std(),
        "max_abs": error.abs().max(),
        "p95_abs": error.abs().quantile(0.95),
        "p99_abs": error.abs().quantile(0.99),
    }


def plot_channel(merged, channel, label, unit, limit, out_path, title):
    """Ground truth vs radar, with the error beneath it."""
    figure, (top, bottom) = plt.subplots(
        2, 1, figsize=(11, 6), sharex=True,
        gridspec_kw={"height_ratios": [2, 1]})
    figure.suptitle(title, fontsize=12, fontweight="bold")

    # Elapsed seconds from the start of the run: absolute unix time forces
    # matplotlib into an offset label that nobody can read.
    time = merged["timestamp_gt"] - merged["timestamp_gt"].min()
    truth = merged[f"{channel}_gt"]
    radar = merged[f"{channel}_radar"]

    if limit is not None:
        top.fill_between(time, truth - limit, truth + limit, color="0.85",
                         label=f"tolerance ±{limit:g} {unit}")
    top.plot(time, truth, color="black", lw=1.4, label="ground truth")
    top.plot(time, radar, color="tab:red", lw=1.2, label="radar")
    top.set_ylabel(f"{label} ({unit})")
    top.legend(fontsize=8)
    top.grid(alpha=0.3)

    error = radar - truth
    bottom.axhline(0, color="black", lw=0.8)
    if limit is not None:
        bottom.axhspan(-limit, limit, color="0.85")
    bottom.plot(time, error, color="tab:red", lw=1.0)
    bottom.set_ylabel(f"error ({unit})")
    bottom.set_xlabel("time (s)")
    bottom.grid(alpha=0.3)

    figure.tight_layout()
    figure.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(figure)


def write_report(path, context, detection, stats, tolerances):
    bar = "=" * 68
    with open(path, "w") as out:
        out.write(f"RADAR KINEMATICS vs GROUND TRUTH\n{bar}\n")
        for key, value in context.items():
            out.write(f"  {key:<22}: {value}\n")
        out.write(f"{bar}\n\nDETECTION\n{'-' * 68}\n")
        for key, value in detection.items():
            if isinstance(value, float):
                out.write(f"  {key:<24}: {value:.2f}\n")
            else:
                out.write(f"  {key:<24}: {value}\n")

        out.write(f"\nACCURACY\n{'-' * 68}\n")
        for channel, label, unit in CHANNELS:
            entry = stats.get(channel)
            if not entry:
                out.write(f"\n  {label}: no overlapping samples\n")
                continue
            limit = tolerances.get(channel) if tolerances else None
            out.write(f"\n  {label}  ({unit})\n")
            out.write(f"    samples              : {entry['n']}\n")
            out.write(f"    mean error           : {entry['mean_error']:>10.4f}\n")
            out.write(f"    mean absolute error  : {entry['mae']:>10.4f}\n")
            out.write(f"    RMS error            : {entry['rmse']:>10.4f}\n")
            out.write(f"    std deviation        : {entry['std']:>10.4f}\n")
            out.write(f"    max absolute error   : {entry['max_abs']:>10.4f}\n")
            out.write(f"    95th pct abs error   : {entry['p95_abs']:>10.4f}\n")
            out.write(f"    99th pct abs error   : {entry['p99_abs']:>10.4f}\n")
            if limit is None:
                out.write("    verdict              : not assessed "
                          "(no tolerance configured)\n")
            else:
                within = 100.0 * (entry['p95_abs'] <= limit)
                out.write(f"    tolerance            : +/-{limit:g}\n")
                out.write(f"    verdict              : "
                          f"{'PASS' if entry['max_abs'] <= limit else 'FAIL'}"
                          f"  (max |error| vs tolerance)\n")

        if not tolerances:
            out.write(f"\n{bar}\nNO TOLERANCES CONFIGURED - no pass/fail was "
                      f"assessed.\nThe percentiles above are the right input "
                      f"for writing a requirement.\nDo not then grade this "
                      f"same run against a limit derived from it.\n{bar}\n")
        out.write("\nEND OF REPORT\n")


def process(path, args, tolerances):
    objects = pd.read_csv(path)
    required = {"source", "frame_num", "timestamp", "distX"}
    if not required.issubset(objects.columns):
        print(f"  SKIPPED {path}: not a long-format objects log")
        return

    merged, chosen = align(objects, args.source, args.truth, args.track_id)
    if merged.empty:
        print(f"  SKIPPED {path}: no overlap between "
              f"'{args.source}' and '{args.truth}'")
        return

    stem = os.path.splitext(os.path.basename(path))[0]
    out_dir = os.path.join(os.path.dirname(os.path.abspath(path)),
                           f"{stem}_report")
    os.makedirs(out_dir, exist_ok=True)

    truth_frames = objects[objects.source == args.truth].frame_num.unique()
    detection = detection_metrics(merged, truth_frames)
    stats = {c: channel_stats(merged, c) for c, _, _ in CHANNELS}

    scenario = args.scenario or "unspecified"
    context = {
        "source log": os.path.basename(path),
        "scenario": f"{scenario}  ({SCENARIOS.get(scenario, 'not a known label')})",
        "radar source": args.source + (f" (id {chosen})" if chosen is not None else ""),
        "ground truth": args.truth,
        "tolerances": args.tolerances or "none configured",
    }

    title_base = f"{scenario} - {args.source} vs {args.truth}"
    for channel, label, unit in CHANNELS:
        limit = tolerances.get(channel) if tolerances else None
        plot_channel(merged, channel, label, unit, limit,
                     os.path.join(out_dir, f"{stem}_{channel}.png"),
                     f"{title_base}  -  {label}")

    report_path = os.path.join(out_dir, f"{stem}_report.txt")
    write_report(report_path, context, detection, stats, tolerances)

    print(f"  {os.path.basename(path)}: {detection['frames_detected']}/"
          f"{detection['frames_with_truth']} frames detected "
          f"({detection['detection_rate']:.1f}%), "
          f"distX MAE {stats['distX']['mae']:.3f} m")
    print(f"    -> {out_dir}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+", help="objects_*.csv file(s)")
    parser.add_argument("--source", default="dynamic",
                        help="Radar source to assess: dynamic, static or track "
                             "(default: dynamic)")
    parser.add_argument("--truth", default="hil1",
                        help="Ground-truth source (default: hil1)")
    parser.add_argument("--track-id", type=float, default=None,
                        help="With --source track, assess this identity "
                             "(default: the longest-lived one)")
    parser.add_argument("--scenario", default=None,
                        help="Test case label, e.g. CCRs, CPNA")
    parser.add_argument("--tolerances", default=None,
                        help="JSON file of per-channel limits. Without it no "
                             "pass/fail is assessed.")
    args = parser.parse_args()

    tolerances = {}
    if args.tolerances:
        with open(args.tolerances) as handle:
            # Keys beginning with "_" are notes in the file, not channels
            tolerances = {k: v for k, v in json.load(handle).items()
                          if not k.startswith("_")}
        print(f"Tolerances: {tolerances}")
    else:
        print("No tolerances configured - reporting measured error only, "
              "no pass/fail.")

    paths = [p for pattern in args.logs for p in sorted(glob.glob(pattern))]
    if not paths:
        raise SystemExit("No matching logs")

    for path in paths:
        process(path, args, tolerances)


if __name__ == "__main__":
    main()
