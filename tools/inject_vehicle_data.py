# -*- coding: utf-8 -*-
"""
Feed the radar the vehicle data it actually listens for.

The radar takes ego speed from J1939 CCVS on extended ID 0x18FEF111, NOT from
0x3F5. Both DBCs document 0x3F5 as ego speed, but the firmware's handler for
it is commented out (mss_main.c:810) and g_f_veh_speed is written only from
the CCVS case at mss_main.c:821.

Without ego speed the radar degenerates: relative velocity is reported as
-ego (so zero), every object classifies as stationary, all six alarm
thresholds stay zero, and time-to-collision divides by zero. Injecting CCVS
is therefore the first thing to fix on a bench that has no vehicle attached.

Encoding, taken from the firmware:

    g_f_veh_speed = ((rxData[1] | (rxData[2] << 8)) * 0.00391) * 0.278

which is J1939 SPN 84, wheel-based vehicle speed: bytes 1-2, little-endian,
1/256 km/h per LSB, converted here to m/s.

Usage:
    python tools/inject_vehicle_data.py --speed 11.1
    python tools/inject_vehicle_data.py --speed 40 --units kmh --yaw 2.5
    python tools/inject_vehicle_data.py --ramp 0:20 --duration 60
    python tools/inject_vehicle_data.py --source socketcan --channel can0 --speed 11.1
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import can_source

# J1939 CCVS1, PGN 65265, source address 0x11 — a 29-bit extended identifier.
CCVS_ID = 0x18FEF111
CCVS_PERIOD_S = 0.1          # CCVS1 is a 100 ms message

YAW_ID = 0x3F7
YAW_PERIOD_S = 0.02

# Firmware constants, reproduced exactly so the round-trip can be asserted.
KMH_PER_LSB = 0.00391        # ~1/256 km/h
MPS_PER_KMH = 0.278


def encode_ccvs(speed_mps):
    """J1939 CCVS frame carrying wheel-based vehicle speed in bytes 1-2."""
    kmh = speed_mps / MPS_PER_KMH
    raw = int(round(kmh / KMH_PER_LSB))
    raw = max(0, min(raw, 0xFFFF))
    data = bytearray(8)
    data[0] = 0xFF               # two-speed switch etc: not available
    data[1] = raw & 0xFF
    data[2] = (raw >> 8) & 0xFF
    for i in range(3, 8):
        data[i] = 0xFF           # J1939 "not available" for unused SPNs
    return bytes(data)


def decode_ccvs(data):
    """The firmware's own decode, for verifying what we just encoded."""
    return ((data[1] | (data[2] << 8)) * KMH_PER_LSB) * MPS_PER_KMH


def encode_yaw(yaw_rate):
    """0x3F7: sign at bit 35, magnitude bits 36-49 at 0.01 (DBC V6)."""
    buffer = bytearray(8)
    magnitude = min(abs(int(round(yaw_rate / 0.01))), 0x3FFF)
    if yaw_rate < 0:
        buffer[4] |= 1 << 3
    buffer[4] |= (magnitude & 0x0F) << 4
    buffer[5] = (magnitude >> 4) & 0xFF
    buffer[6] |= (magnitude >> 12) & 0x03
    return bytes(buffer)


def speed_at(elapsed, args):
    if args.ramp is None:
        return args.speed
    start, end = args.ramp
    if args.duration <= 0:
        return end
    fraction = min(1.0, elapsed / args.duration)
    return start + (end - start) * fraction


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default=None,
                        help="kvaser | socketcan | virtual (default: the "
                             "platform default)")
    parser.add_argument("--channel", default="0", help="CAN channel")
    parser.add_argument("--bitrate", type=int, default=500000)
    parser.add_argument("--speed", type=float, default=11.1,
                        help="Constant ego speed (default: 11.1 m/s = 40 km/h)")
    parser.add_argument("--units", choices=("mps", "kmh"), default="mps",
                        help="Units for --speed and --ramp (default: mps)")
    parser.add_argument("--ramp", metavar="FROM:TO",
                        help="Ramp speed over --duration instead of holding it")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Seconds to run; 0 runs until interrupted")
    parser.add_argument("--yaw", type=float, default=None,
                        help="Also send 0x3F7 yaw rate, in the DBC's units")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show the frames that would be sent, send nothing")
    args = parser.parse_args()

    to_mps = (lambda v: v) if args.units == "mps" else (lambda v: v * MPS_PER_KMH)
    args.speed = to_mps(args.speed)
    if args.ramp:
        start, end = (to_mps(float(v)) for v in args.ramp.split(":"))
        args.ramp = (start, end)
        if args.duration <= 0:
            parser.error("--ramp needs --duration")
    else:
        args.ramp = None

    # Prove the encoding against the firmware's own arithmetic before sending.
    sample = encode_ccvs(args.speed if args.ramp is None else args.ramp[1])
    target = args.speed if args.ramp is None else args.ramp[1]
    print(f"CCVS 0x{CCVS_ID:08X} (29-bit extended), every {CCVS_PERIOD_S*1000:.0f} ms")
    print(f"  {target:.2f} m/s ({target/MPS_PER_KMH:.1f} km/h) encodes to "
          + " ".join("%02X" % b for b in sample))
    print(f"  firmware would decode that as {decode_ccvs(sample):.3f} m/s")

    if args.dry_run:
        print("\n--dry-run: nothing sent.")
        return 0

    try:
        bus = can_source.open_source(args.source, channel=args.channel,
                                     bitrate=args.bitrate)
    except Exception as e:
        print(f"\nCould not open the CAN interface: {e}")
        return 1

    import can
    print(f"\nSending on {getattr(bus, 'channel_info', args.source)} — Ctrl+C to stop")

    started = time.monotonic()
    next_ccvs = started
    next_yaw = started
    sent = 0
    try:
        while True:
            now = time.monotonic()
            elapsed = now - started
            if args.duration and elapsed >= args.duration:
                break

            if now >= next_ccvs:
                speed = speed_at(elapsed, args)
                bus.send(can.Message(arbitration_id=CCVS_ID,
                                     data=encode_ccvs(speed),
                                     is_extended_id=True))
                sent += 1
                next_ccvs += CCVS_PERIOD_S
                if sent % 10 == 1:
                    print(f"  t={elapsed:6.1f}s  ego {speed:5.2f} m/s "
                          f"({speed/MPS_PER_KMH:5.1f} km/h)")

            if args.yaw is not None and now >= next_yaw:
                bus.send(can.Message(arbitration_id=YAW_ID,
                                     data=encode_yaw(args.yaw),
                                     is_extended_id=False))
                next_yaw += YAW_PERIOD_S

            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.shutdown()
    print(f"Sent {sent:,} CCVS frames.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
