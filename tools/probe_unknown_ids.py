# -*- coding: utf-8 -*-
"""
Reverse-engineer the undecoded CAN messages in a raw ORDLAV log.

The DBC documents 0x334-0x338, 0x500 and 0x6F0 as byte-per-signal placeholders
with unknown layout, and the radar publishes no object ID, orientation,
classification or size in any documented message. Those unknown blocks are the
only place that information could be hiding.

Rather than eyeball hex, this characterises every byte of each unknown message
across a whole run:

  * which bits ever change, and which are stuck
  * bytes that behave like counters (steady +1), like slot indices (constant
    per ID but different across a group), or like flags (few distinct values)
  * 8- and 16-bit fields whose value tracks a reference signal - by default the
    longitudinal distance from 0x510 - which is how a position field gives
    itself away

Correlation is evidence, not proof: confirm any candidate against a second log
with a different target geometry before trusting it.

Usage:
    python tools/probe_unknown_ids.py bench_log.csv
    python tools/probe_unknown_ids.py bench_log.csv --ids 334,335,336
    python tools/probe_unknown_ids.py bench_log.csv --ref-id 590   # static object
"""

import argparse
import csv
import math
from collections import defaultdict

# Messages whose layout is already known, skipped unless --all is given.
DECODED_IDS = {0x510, 0x590, 0x6F1, 0x001, 0x3F5, 0x100, 0x101, 0x501, 0x581}
# The tunnelled mmWave byte stream: not CAN signals, analysed separately.
RADAR_STREAM_IDS = {0xC1, 0xD1, 0xD2, 0xD3, 0xE1}

MIN_SAMPLES = 8


def load(path):
    """Read a raw log into {arbitration_id: [(timestamp, bytes), ...]}."""
    messages = defaultdict(list)
    with open(path, newline="") as handle:
        for row in csv.reader(handle):
            if len(row) < 4:
                continue
            try:
                timestamp = float(row[0])
                can_id = int(row[1], 16)
                data = bytes.fromhex(row[3].replace(" ", ""))
            except ValueError:
                continue
            messages[can_id].append((timestamp, data))
    return messages


def reference_series(messages, ref_id):
    """Longitudinal distance over time, decoded from an object message."""
    series = []
    for timestamp, data in messages.get(ref_id, []):
        if len(data) < 2:
            continue
        raw = data[0] | ((data[1] & 0b00011111) << 8)
        series.append((timestamp, 0.032 * raw - 60.8))
    return series


def nearest(series, timestamp):
    """Reference value closest in time; the sources are not synchronised."""
    best, best_dt = None, None
    for ts, value in series:
        dt = abs(ts - timestamp)
        if best_dt is None or dt < best_dt:
            best, best_dt = value, dt
    return best


def pearson(xs, ys):
    n = len(xs)
    if n < MIN_SAMPLES:
        return 0.0
    mx, my = sum(xs) / n, sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    dx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    dy = math.sqrt(sum((y - my) ** 2 for y in ys))
    return num / (dx * dy) if dx and dy else 0.0


def looks_like_counter(values):
    """Steady +1 steps, allowing for wraps and the odd dropped frame."""
    if len(values) < MIN_SAMPLES:
        return False
    steps = [(b - a) % 256 for a, b in zip(values, values[1:])]
    return sum(1 for s in steps if s == 1) >= 0.8 * len(steps)


def describe_byte(index, values):
    distinct = sorted(set(values))
    changing_bits = 0
    for value in values:
        changing_bits |= value ^ values[0]

    if len(distinct) == 1:
        kind = f"constant 0x{distinct[0]:02X}"
    elif looks_like_counter(values):
        kind = "COUNTER (+1 per message)"
    elif len(distinct) <= 4:
        kind = f"flag/enum {[hex(v) for v in distinct]}"
    elif len(distinct) > 64:
        kind = "wide-ranging (data-like)"
    else:
        kind = f"{len(distinct)} distinct values"

    return (f"    byte {index}: {kind:<34} "
            f"range {min(values):>3}..{max(values):<3} "
            f"bits changing: {changing_bits:08b}")


def candidate_fields(records, reference):
    """Fields whose value tracks the reference signal."""
    if not reference:
        return []

    width = min(len(d) for _, d in records)
    paired = [(d, nearest(reference, t)) for t, d in records]
    paired = [(d, r) for d, r in paired if r is not None]
    if len(paired) < MIN_SAMPLES:
        return []

    refs = [r for _, r in paired]
    found = []

    # A free-running counter correlates with any reference that moves steadily
    # in one direction - a closing target does exactly that. Exclude counter
    # bytes so they do not drown out real position fields.
    counters = {i for i in range(width)
                if looks_like_counter([d[i] for _, d in records])}

    for i in range(width):
        if i in counters:
            continue
        values = [d[i] for d, _ in paired]
        if len(set(values)) > 2:
            r = pearson(values, refs)
            if abs(r) > 0.8:
                found.append((abs(r), f"byte {i} (u8)", r))

    for i in range(width - 1):
        if i in counters or i + 1 in counters:
            continue
        for label, extract in (
            ("u16 LE", lambda d, i=i: d[i] | (d[i + 1] << 8)),
            ("u16 BE", lambda d, i=i: (d[i] << 8) | d[i + 1]),
        ):
            values = [extract(d) for d, _ in paired]
            if len(set(values)) > 2:
                r = pearson(values, refs)
                if abs(r) > 0.8:
                    found.append((abs(r), f"bytes {i}-{i+1} ({label})", r))

    found.sort(reverse=True)
    return found[:6]


def report_group(messages, ids):
    """Bytes that are constant within an ID but differ across a group look
    like a slot index - the signature of a per-object message block."""
    present = [i for i in ids if len(messages.get(i, [])) >= MIN_SAMPLES]
    if len(present) < 2:
        return

    width = min(min(len(d) for _, d in messages[i]) for i in present)
    slots = []
    for index in range(width):
        per_id = {}
        for can_id in present:
            values = {d[index] for _, d in messages[can_id]}
            if len(values) != 1:
                break
            per_id[can_id] = values.pop()
        else:
            if len(set(per_id.values())) == len(present):
                slots.append((index, per_id))

    print("\n" + "=" * 72)
    print(f"GROUP ANALYSIS  {', '.join('0x%X' % i for i in present)}")
    print("=" * 72)
    if slots:
        for index, per_id in slots:
            mapping = "  ".join(f"0x{i:X}->{v}" for i, v in sorted(per_id.items()))
            print(f"  byte {index} is constant per ID and unique across the group:")
            print(f"      {mapping}")
            print("      -> looks like a slot/object index")
    else:
        print("  No byte is constant-per-ID and unique across the group.")
        print("  These may not be a per-object block, or the log has too few samples.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", help="Raw CAN log CSV written by the processor")
    parser.add_argument("--ids", help="Comma-separated hex IDs to inspect "
                                      "(default: every undecoded ID present)")
    parser.add_argument("--ref-id", default="510",
                        help="Hex ID of the object message used as the reference "
                             "signal (default: 510)")
    parser.add_argument("--all", action="store_true",
                        help="Include IDs whose layout is already known")
    args = parser.parse_args()

    messages = load(args.log)
    if not messages:
        raise SystemExit(f"No CAN rows read from {args.log}")

    reference = reference_series(messages, int(args.ref_id, 16))
    print(f"Log: {args.log}")
    print(f"{sum(len(v) for v in messages.values()):,} messages, "
          f"{len(messages)} distinct IDs")
    print(f"Reference signal: 0x{int(args.ref_id, 16):X} longitudinal distance, "
          f"{len(reference)} samples"
          + ("" if reference else "  (absent - correlation disabled)"))

    if args.ids:
        wanted = [int(x, 16) for x in args.ids.split(",")]
    else:
        wanted = sorted(i for i in messages
                        if i not in RADAR_STREAM_IDS
                        and (args.all or i not in DECODED_IDS))

    for can_id in wanted:
        records = messages.get(can_id, [])
        if len(records) < MIN_SAMPLES:
            print(f"\n0x{can_id:X}: only {len(records)} messages - skipped")
            continue

        span = records[-1][0] - records[0][0]
        rate = (len(records) - 1) / span if span > 0 else 0
        width = min(len(d) for _, d in records)

        print("\n" + "=" * 72)
        print(f"0x{can_id:X}   {len(records):,} messages   {rate:.1f} Hz   "
              f"{width} bytes")
        print("=" * 72)

        for index in range(width):
            print(describe_byte(index, [d[index] for _, d in records]))

        hits = candidate_fields(records, reference)
        if hits:
            print(f"\n    Fields tracking the reference signal "
                  f"(counter bytes excluded):")
            for _, label, r in hits:
                print(f"      {label:<24} r = {r:+.3f}")
        elif reference:
            print("\n    No field correlates with the reference signal.")

    report_group(messages, [0x334, 0x335, 0x336, 0x337, 0x338])

    print("\nNext: confirm any candidate against a second log with different "
          "target geometry\nbefore decoding it for real.")


if __name__ == "__main__":
    main()
