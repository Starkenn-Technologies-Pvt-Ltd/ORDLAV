# -*- coding: utf-8 -*-
"""
Print what the radar actually reported, decoded straight from a raw CAN log.

Use this when the displayed distance looks wrong. It bypasses the visualizer
entirely and decodes the object messages from the logged bytes, so it settles
whether a suspect value came from the radar or from something downstream.

It also shows the raw payload for the closest frames, so the bytes can be
checked by hand against the DBC.

Usage:
    python tools/decode_objects.py 15_16_10_06_2026.csv
    python tools/decode_objects.py raw_log.csv --id 510 --rows 40
"""

import argparse
import csv
from collections import defaultdict

# DBC V6: shared object layout, Intel bit order
SIGNALS = {
    "distX": (0, 13, 0.032, -60.8),
    "distY": (13, 12, 0.064, -131.072),
    "vrelX": (25, 11, 0.1, -102.4),
    "vrelY": (36, 11, 0.1, -102.4),
}

OBJECT_IDS = {
    0x510: "Dynamic_Object_1", 0x511: "Dynamic_Object_2", 0x512: "Dynamic_Object_3",
    0x590: "Static_Object_1", 0x591: "Static_Object_2", 0x592: "Static_Object_3",
    0x100: "HIL_Target1", 0x101: "HIL_Target2",
    0x102: "HIL_Target3", 0x103: "HIL_Target4",
}


def bits(data, start, length):
    value = 0
    for index in range(length):
        bit = start + index
        if bit // 8 < len(data) and (data[bit // 8] >> (bit % 8)) & 1:
            value |= 1 << index
    return value


def decode(data):
    out = {}
    for name, (start, length, scale, offset) in SIGNALS.items():
        out[name] = scale * bits(data, start, length) + offset
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="Raw CAN log CSV written by the processor")
    parser.add_argument("--id", help="Only this hex ID, e.g. 510")
    parser.add_argument("--rows", type=int, default=15,
                        help="Closest frames to list per ID (default: 15)")
    args = parser.parse_args()

    wanted = {int(args.id, 16)} if args.id else set(OBJECT_IDS)
    records = defaultdict(list)

    with open(args.log, newline="") as handle:
        for row in csv.reader(handle):
            if len(row) < 4:
                continue
            try:
                timestamp = float(row[0])
                can_id = int(row[1], 16)
                data = bytes.fromhex(row[3].replace(" ", ""))
            except ValueError:
                continue
            if can_id in wanted and len(data) >= 6:
                records[can_id].append((timestamp, data))

    if not records:
        raise SystemExit("No object messages found. Present IDs may differ - "
                         "check the log, or pass --id.")

    for can_id in sorted(records):
        entries = records[can_id]
        decoded = [(t, decode(d), d) for t, d in entries]
        distances = [v["distX"] for _, v, _ in decoded]

        print("\n" + "=" * 72)
        print(f"0x{can_id:X}  {OBJECT_IDS.get(can_id, '')}   {len(entries):,} messages")
        print("=" * 72)
        print(f"  distX  min {min(distances):8.3f} m   "
              f"max {max(distances):8.3f} m   "
              f"mean {sum(distances)/len(distances):8.3f} m")

        print(f"\n  Closest {args.rows} frames:")
        print(f"    {'distX':>9} {'distY':>9} {'vrelX':>8} {'vrelY':>8}   raw bytes")
        for _, values, data in sorted(decoded, key=lambda r: r[1]["distX"])[:args.rows]:
            print(f"    {values['distX']:>9.3f} {values['distY']:>9.3f} "
                  f"{values['vrelX']:>8.2f} {values['vrelY']:>8.2f}   "
                  + " ".join("%02X" % b for b in data))

        near = sum(1 for d in distances if d < 1.0)
        print(f"\n  frames reporting under 1 m: {near} of {len(distances)}")
        if not near:
            print("  -> the radar never reported this object inside 1 m")


if __name__ == "__main__":
    main()
