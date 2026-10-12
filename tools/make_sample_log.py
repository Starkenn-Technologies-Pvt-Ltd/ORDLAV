# -*- coding: utf-8 -*-
"""
Synthesize a raw CAN log in the format CANFrameProcessor.log_to_csv writes, so
the pipeline can be exercised end to end without the Kvaser bench.

The scenario is a single vehicle closing from ~80 m to ~12 m while the ego car
travels at ~40 km/h, with a roadside static object, radar point cloud, clusters
and tracks, plus HIL ground-truth targets (0x100/0x101) that carry a small
deliberate error against the radar reports — which is what
Analysis/plot_and_analyse_data.py measures.

Byte layouts below mirror what CAN_Frame_Processor_v1.4.py actually decodes
(notably 14-byte point records and 12-byte track records), so the generated log
is only meaningful against that parser.

Usage:
    python tools/make_sample_log.py                      # 200 frames -> sample_log.csv
    python tools/make_sample_log.py -o my.csv -n 400
"""

import argparse
import csv
import math
import random
import struct
import time

# CAN IDs (kept in sync with CAN_Frame_Processor_v1.4.py)
ID_HEADER = 0xC1
ID_POINT_CLOUD = 0xD1
ID_CLUSTERS = 0xD2
ID_TRACKERS = 0xD3
ID_MOVING_OBJ = 0x510
ID_STATIC_OBJ = 0x590
ID_HIL_1 = 0x100
ID_HIL_2 = 0x101
ID_EGO_SPEED = 0x3F5
ID_ALARM_THRESH = 0x6F1
ID_ALARM_STATUS = 0x001
ID_DYNAMIC_COUNT = 0x501
ID_STATIC_COUNT = 0x581

FRAME_DELIMITER = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])
Q_FORMAT = 7  # values are scaled by 2**7 = 128
FRAME_PERIOD_S = 0.05  # 20 Hz radar


def clamp(value, low, high):
    return max(low, min(high, value))


def encode_object(rng_m, lat_m, v_rng, v_lat):
    """Pack the 6-byte position/velocity layout used by 0x510/0x590/0x100/0x101.

    Field 'x' is longitudinal range, 'y' is lateral offset, matching the
    scaling in parse_510 (x: 0.032 m/LSB - 60.8, y: 0.064 m/LSB - 131.072).
    """
    rx = clamp(int(round((rng_m + 60.8) / 0.032)), 0, 0x1FFF)
    ry = clamp(int(round((lat_m + 131.072) / 0.064)), 0, 0x0FFF)
    rvx = clamp(int(round((v_rng + 102.4) / 0.1)), 0, 0x7FF)
    rvy = clamp(int(round((v_lat + 102.4) / 0.1)), 0, 0x7FF)
    return bytes([
        rx & 0xFF,
        ((rx >> 8) & 0x1F) | ((ry & 0x07) << 5),
        (ry >> 3) & 0xFF,
        ((ry >> 11) & 0x01) | ((rvx & 0x7F) << 1),
        ((rvx >> 7) & 0x0F) | ((rvy & 0x0F) << 4),
        (rvy >> 4) & 0x7F,
        0x00,
        0x00,
    ])


def encode_ego_speed(kmph):
    raw = clamp(int(round(kmph / 0.01)), 0, 0x7FFF)
    return bytes([raw & 0xFF, (raw >> 8) & 0x7F, 0, 0, 0, 0, 0, 0])


# DBC: FCWS_Alarm_Distance_Thresh is 0.5 m per LSB
ALARM_THRESH_SCALE = 0.5


def encode_alarm_thresholds(static_levels_m, moving_levels_m):
    """Levels are given in metres and packed at 0.5 m/LSB (byte 3 unused)."""
    raw = [clamp(int(round(v / ALARM_THRESH_SCALE)), 0, 255)
           for v in list(static_levels_m) + list(moving_levels_m)]
    s1, s2, s3, m1, m2, m3 = raw
    return bytes([s1, s2, s3, 0x00, m1, m2, m3, 0x00])


def encode_count(count):
    """Object count messages (0x501 / 0x581) carry the count in byte 0."""
    return bytes([clamp(int(count), 0, 255), 0, 0, 0, 0, 0, 0, 0])


def q(value):
    """Scale a real value into the radar's Q-format signed 16-bit form."""
    return clamp(int(round(value * (1 << Q_FORMAT))), -32768, 32767)


def build_radar_payload(frame_num, points, clusters, tracks):
    """Build the mmWave packet that follows the 8-byte barker code.

    Returns the bytes after the delimiter: a 32-byte header remainder plus
    TLV 1 (point cloud), TLV 2 (clusters) and TLV 3 (tracks), in that order.
    TLVs must be contiguous — the parser walks them by offset.
    """
    tlv1_body = struct.pack("<HH", len(points), Q_FORMAT)
    for x, y, doppler in points:
        # doppler, peakVal, x, y, then 6 bytes the parser skips (z + spare)
        tlv1_body += struct.pack("<hHhh", q(doppler), 480, q(x), q(y))
        tlv1_body += b"\x00" * 6
    tlv1 = struct.pack("<II", 1, len(tlv1_body)) + tlv1_body

    tlv2_body = struct.pack("<HH", len(clusters), Q_FORMAT)
    for x, y, xsize, ysize in clusters:
        tlv2_body += struct.pack("<hhhh", q(x), q(y), q(xsize), q(ysize))
    tlv2 = struct.pack("<II", 2, len(tlv2_body)) + tlv2_body

    tlv3_body = struct.pack("<HH", len(tracks), Q_FORMAT)
    for x, y, vx, vy in tracks:
        tlv3_body += struct.pack("<hhhh", q(x), q(y), q(vx), q(vy))
        tlv3_body += b"\x00" * 4  # spare the parser skips
    tlv3 = struct.pack("<II", 3, len(tlv3_body)) + tlv3_body

    tlvs = tlv1 + tlv2 + tlv3
    header_rest = struct.pack(
        "<IIIIIII",
        0x0000BEEF,          # CRC (unchecked by the parser)
        40 + len(tlvs),      # total message length
        0x000A1642,          # platform
        frame_num,
        frame_num * 1000,    # CPU time
        len(points),         # numDetectedObj
        3,                   # numTLVs
    ) + struct.pack("<I", 0)  # sub-frame number
    return header_rest, tlvs


def chunk_to_messages(header_rest, tlvs):
    """Split the radar payload into 8-byte CAN messages, tagged by region."""
    blob = header_rest + tlvs
    # Region boundaries are cosmetic (all four IDs feed one buffer), but keep
    # them plausible: header on C1, then point/cluster/track TLVs.
    boundaries = [
        (len(header_rest), ID_HEADER),
        (len(blob), ID_POINT_CLOUD),
    ]
    messages = []
    for offset in range(0, len(blob), 8):
        chunk = blob[offset:offset + 8].ljust(8, b"\x00")
        can_id = next(cid for limit, cid in boundaries if offset < limit)
        messages.append((can_id, chunk))
    return messages


def scenario_frame(i, total):
    """Ground truth and radar reports for frame ``i``."""
    progress = i / max(1, total - 1)

    # Closing target: 80 m -> 12 m, weaving slightly in lateral offset
    true_rng = 80.0 - 68.0 * progress
    true_lat = 2.0 * math.sin(progress * math.pi * 2.0)
    true_v_rng = -13.6  # ~49 km/h closing speed
    true_v_lat = 0.4 * math.cos(progress * math.pi * 2.0)

    # Radar reports the same target with a small, realistic error
    rep_rng = true_rng + random.gauss(0.0, 0.25)
    rep_lat = true_lat + random.gauss(0.0, 0.12)
    rep_v_rng = true_v_rng + random.gauss(0.0, 0.3)
    rep_v_lat = true_v_lat + random.gauss(0.0, 0.1)

    # Roadside static object, closing only because the ego car moves
    static_rng = 90.0 - 70.0 * progress
    static_lat = 5.2 + random.gauss(0.0, 0.08)

    ego_kmph = 40.0 + 2.0 * math.sin(progress * math.pi * 3.0)

    # Point cloud: returns off the target plus road clutter
    points = []
    for _ in range(14):
        points.append((
            rep_lat + random.gauss(0.0, 0.5),
            rep_rng + random.gauss(0.0, 0.8),
            true_v_rng + random.gauss(0.0, 0.4),
        ))
    for _ in range(6):
        points.append((
            random.uniform(-12.0, 12.0),
            random.uniform(5.0, 120.0),
            random.gauss(0.0, 1.5),
        ))

    clusters = [
        (rep_lat, rep_rng, 1.8, 4.2),
        (static_lat, static_rng, 1.2, 1.2),
    ]

    tracks = [(rep_lat, rep_rng, rep_v_lat, rep_v_rng)]
    if progress > 0.3:  # a second track appears part-way through
        tracks.append((rep_lat - 3.4, rep_rng + 18.0, 0.1, -2.0))

    # A world-stationary object is measured closing at the ego speed; it does
    # not report zero relative velocity.
    static_v_rng = -ego_kmph / 3.6

    return {
        "report": (rep_rng, rep_lat, rep_v_rng, rep_v_lat),
        "truth": (true_rng, true_lat, true_v_rng, true_v_lat),
        "static": (static_rng, static_lat, static_v_rng, 0.0),
        "ego_kmph": ego_kmph,
        "points": points,
        "clusters": clusters,
        "tracks": tracks,
    }


def generate(path, frames, seed):
    random.seed(seed)
    base_ts = time.time()
    rows = []

    def emit(ts, can_id, payload):
        rows.append([
            "%.6f" % ts,
            format(can_id, "X"),
            len(payload),
            " ".join("%02X" % b for b in payload),
        ])

    for i in range(frames):
        frame_ts = base_ts + i * FRAME_PERIOD_S
        state = scenario_frame(i, frames)
        rng, lat, v_rng, v_lat = state["report"]
        t_rng, t_lat, t_v_rng, t_v_lat = state["truth"]
        s_rng, s_lat, s_v_rng, s_v_lat = state["static"]

        # Messages inside a frame are spaced so the whole frame fits its period
        header_rest, tlvs = build_radar_payload(
            i + 1, state["points"], state["clusters"], state["tracks"])
        radar_messages = chunk_to_messages(header_rest, tlvs)

        signal_messages = [
            (ID_MOVING_OBJ, encode_object(rng, lat, v_rng, v_lat)),
            (ID_STATIC_OBJ, encode_object(s_rng, s_lat, s_v_rng, s_v_lat)),
            (ID_HIL_1, encode_object(t_rng, t_lat, t_v_rng, t_v_lat)),
            (ID_HIL_2, encode_object(s_rng, s_lat, s_v_rng, s_v_lat)),
            (ID_EGO_SPEED, encode_ego_speed(state["ego_kmph"])),
            (ID_ALARM_THRESH, encode_alarm_thresholds((30, 20, 10), (60, 40, 20))),
            # Counts the radar reports; the second track appears part-way in
            (ID_DYNAMIC_COUNT, encode_count(len(state["tracks"]))),
            (ID_STATIC_COUNT, encode_count(1)),
        ]
        # Alarm only fires once the target is inside the level-1 moving threshold
        if rng < 60.0:
            level = 1 if rng > 40.0 else (2 if rng > 20.0 else 3)
            signal_messages.append((ID_ALARM_STATUS, bytes([level, 0, 0, 0, 0, 0, 0, 0])))

        frame_messages = ([(ID_HEADER, FRAME_DELIMITER)]
                          + signal_messages
                          + radar_messages)
        spacing = FRAME_PERIOD_S / (len(frame_messages) + 1)
        for slot, (can_id, payload) in enumerate(frame_messages):
            emit(frame_ts + slot * spacing, can_id, payload)

    # A trailing delimiter so the last frame is closed and queued
    emit(base_ts + frames * FRAME_PERIOD_S, ID_HEADER, FRAME_DELIMITER)

    with open(path, "w", newline="") as handle:
        csv.writer(handle).writerows(rows)

    return len(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-o", "--output", default="sample_log.csv",
                        help="Output CSV path (default: sample_log.csv)")
    parser.add_argument("-n", "--frames", type=int, default=200,
                        help="Number of radar frames (default: 200 = 10 s @ 20 Hz)")
    parser.add_argument("--seed", type=int, default=7, help="RNG seed (default: 7)")
    args = parser.parse_args()

    count = generate(args.output, args.frames, args.seed)
    print(f"Wrote {count:,} CAN messages across {args.frames} frames -> {args.output}")
    print(f"Replay it with:\n"
          f"  python CAN_Frame_Processor_v1.4.py --source replay:{args.output} --no-raw-log")


if __name__ == "__main__":
    main()
