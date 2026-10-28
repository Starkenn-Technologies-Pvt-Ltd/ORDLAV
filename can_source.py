# -*- coding: utf-8 -*-
"""
Pluggable CAN message sources for the ORDLAV pipeline.

The frame processor only needs an object that exposes ``recv(timeout)`` and
``shutdown()``, where ``recv`` returns something carrying ``timestamp``,
``arbitration_id``, ``dlc`` and ``data``. That is exactly the python-can Bus
contract, so real hardware is handed through untouched and replay is a drop-in
stand-in for it.

Sources (``--source`` / ``$ORDLAV_SOURCE``):

    kvaser            Kvaser hardware (the lab setup; Windows/Linux only)
    virtual           python-can in-process virtual bus (no hardware)
    replay:<file>     re-play a raw CSV log written by CANFrameProcessor.log_to_csv
    <other>           any python-can interface name, e.g. socketcan, pcan, slcan

Replay reproduces the original inter-message timing from the log timestamps,
scaled by ``speed`` (0 = as fast as possible).
"""

import csv
import os
import sys
import time

import can

# Import what we actually use rather than reaching through the package at call
# time. PyPI hosts an unrelated project also called "can", so `pip install can`
# (instead of python-can) yields a module without any of this - better to find
# out at import, with a message that says what to do, than to fail later with
# "module 'can' has no attribute 'interface'".
try:
    from can.interface import Bus as CanBus
    from can import Message as CanMessage
except (ImportError, AttributeError) as exc:
    CanBus = None
    CanMessage = None
    _CAN_IMPORT_ERROR = exc
else:
    _CAN_IMPORT_ERROR = None


def describe_can_package():
    """Where the imported 'can' came from, for diagnostics."""
    return (f"'can' imported from {getattr(can, '__file__', '<unknown>')} "
            f"(version {getattr(can, '__version__', 'unknown')})")


def require_python_can():
    """Raise with remediation if the 'can' on this machine is not python-can."""
    if CanBus is not None:
        return
    raise ImportError(
        "python-can is required but could not be imported ({0}).\n"
        "  {1}\n"
        "  PyPI also hosts an unrelated package named 'can'. If that one got "
        "installed, replace it:\n"
        "      pip uninstall -y can\n"
        "      pip install -r requirements.txt\n"
        "  Then check with:  python -c \"import can; print(can.__version__, can.__file__)\""
        .format(_CAN_IMPORT_ERROR, describe_can_package())
    )


# A gap longer than this in the log is collapsed, so a recording with a long
# idle stretch does not stall the replay.
MAX_REPLAY_GAP_S = 0.5

# Backends that take their bus speed from the OS network interface rather than
# from python-can.
BITRATE_SET_ELSEWHERE = {"socketcan", "socketcand", "virtual"}

# Fallback spacing when a log has unusable timestamps.
DEFAULT_REPLAY_INTERVAL_S = 0.001


def default_source():
    """Pick a sensible source for this machine.

    Kvaser drivers only exist for Windows and Linux, so a Mac defaults to the
    virtual bus instead of failing at import time.
    """
    env = os.environ.get("ORDLAV_SOURCE")
    if env:
        return env
    if sys.platform.startswith(("win", "linux")):
        return "kvaser"
    return "virtual"


class CsvReplayBus:
    """Replays a raw CAN log CSV with the cadence it was recorded at.

    The log format is the one ``log_to_csv`` writes, with no header row:
    ``timestamp, arbitration_id (hex, no 0x), dlc, "AA BB CC ..."``.
    A header row is tolerated and skipped.
    """

    def __init__(self, path, speed=1.0, loop=False, can_filters=None):
        require_python_can()
        self.path = path
        self.speed = float(speed)
        self.loop = bool(loop)
        self.channel_info = "csv-replay:%s" % os.path.basename(path)
        self.exhausted = False

        allowed = None
        if can_filters:
            allowed = {f["can_id"] for f in can_filters}

        self._messages = self._load(path, allowed)
        if not self._messages:
            raise ValueError(
                "No usable CAN rows in %s (after ID filtering)" % path
            )

        # Absolute delivery schedule: offsets from the first message, with
        # long idle gaps collapsed. Pacing off a precomputed schedule keeps
        # the overall rate accurate -- sleeping per message instead would be
        # dominated by the OS sleep granularity (~1 ms) at high speeds.
        self._offsets = self._build_schedule(self._messages)

        self._idx = 0
        self._start = None
        self.laps = 0

    @staticmethod
    def _build_schedule(messages):
        offsets = [0.0]
        for previous, current in zip(messages, messages[1:]):
            dt = current.timestamp - previous.timestamp
            if dt <= 0 or dt > MAX_REPLAY_GAP_S:
                dt = DEFAULT_REPLAY_INTERVAL_S
            offsets.append(offsets[-1] + dt)
        return offsets

    # ------------------------------------------------------------------ load

    @staticmethod
    def _load(path, allowed):
        messages = []
        skipped = 0
        with open(path, "r", newline="") as handle:
            for row in csv.reader(handle):
                if len(row) < 4:
                    continue
                try:
                    timestamp = float(row[0])
                except ValueError:
                    continue  # header row or junk line
                try:
                    arbitration_id = int(row[1], 16)
                    dlc = int(row[2])
                    data = bytes.fromhex(row[3].replace(" ", ""))
                except ValueError:
                    skipped += 1
                    continue
                if allowed is not None and arbitration_id not in allowed:
                    continue
                messages.append(
                    CanMessage(
                        timestamp=timestamp,
                        arbitration_id=arbitration_id,
                        dlc=dlc or len(data),
                        data=data,
                        is_extended_id=False,
                        check=False,
                    )
                )
        if skipped:
            print("WARNING: skipped %d malformed row(s) in %s" % (skipped, path))
        return messages

    # ------------------------------------------------------------------ recv

    def recv(self, timeout=None):
        if self._idx >= len(self._messages):
            if not self.loop:
                self.exhausted = True
                if timeout:
                    time.sleep(timeout)  # keep the caller's poll loop calm
                return None
            self._idx = 0
            self.laps += 1
            self._start = None

        now = time.monotonic()
        if self._start is None:
            self._start = now

        if self.speed > 0:
            due = self._start + self._offsets[self._idx] / self.speed
            wait = due - now
            if wait > 0:
                if timeout is not None and wait > timeout:
                    time.sleep(timeout)
                    return None  # not due yet; stay responsive to shutdown
                time.sleep(wait)

        message = self._messages[self._idx]
        self._idx += 1
        return message

    def shutdown(self):
        self._idx = len(self._messages)

    # python-can Buses are context managers; mirror that.
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.shutdown()


def _explain_open_failure(error):
    """Turn python-can's internal failure into something actionable.

    A missing driver surfaces as a NameError for a canlib symbol that was
    never bound, which says nothing useful to whoever is standing at the
    bench.
    """
    text = f"{type(error).__name__}: {error}".lower()
    if "canget" in text or "canlib" in text or isinstance(error, NameError):
        return ("The Kvaser driver (canlib) is not installed or not loadable "
                "on this machine. Kvaser supplies drivers for Windows and "
                "Linux only - there is none for macOS.")
    if "channel" in text:
        return ("The driver loaded but that channel does not exist. Try "
                "--channel 0 or 1, and check the device is plugged in.")
    if "bitrate" in text or "param" in text:
        return "The interface rejected the bitrate; confirm the bus speed."
    return ("Check the device is connected, its drivers are installed, and "
            "nothing else already has the channel open.")


def describe_available(channel="0", bitrate=500000):
    """Report what CAN hardware this machine can actually reach.

    Answers "is the radar plugged in and visible to us" without starting a
    capture, which otherwise means reading a wall of python-can import
    warnings and guessing.
    """
    require_python_can()
    lines = [f"python-can {getattr(can, '__version__', '?')} on {sys.platform}",
             f"  {describe_can_package()}", ""]

    try:
        configs = can.detect_available_configs()
    except Exception as e:
        configs = []
        lines.append(f"  interface detection failed: {e}")

    real = [c for c in configs
            if c.get("interface") not in ("virtual", "udp_multicast")]
    lines.append("Detected interfaces:")
    if configs:
        for config in configs:
            kind = "hardware" if config in real else "software only"
            lines.append(f"  {config.get('interface'):<16} "
                         f"channel {config.get('channel')}   ({kind})")
    else:
        lines.append("  none")

    # Probe the bench interface directly: detection does not always list a
    # device that will nonetheless open.
    lines.append("")
    lines.append(f"Opening 'kvaser' channel {channel} at {bitrate} bit/s:")
    try:
        bus = CanBus(interface="kvaser", channel=channel, bitrate=bitrate)
    except Exception as e:
        lines.append(f"  FAILED - {type(e).__name__}: {e}")
        lines.append(f"  -> {_explain_open_failure(e)}")
    else:
        lines.append(f"  OK - {getattr(bus, 'channel_info', 'opened')}")
        lines.append("  Listening 3 s for traffic...")
        seen = {}
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            message = bus.recv(timeout=0.2)
            if message is not None:
                seen[message.arbitration_id] = seen.get(message.arbitration_id, 0) + 1
        bus.shutdown()
        if seen:
            lines.append(f"  {sum(seen.values()):,} messages from "
                         f"{len(seen)} IDs:")
            for can_id, count in sorted(seen.items()):
                lines.append(f"    0x{can_id:03X}  {count:,}")
        else:
            lines.append("  Bus opened but NO traffic in 3 s - the interface "
                         "is present but the radar is not transmitting "
                         "(check power, wiring and bitrate).")

    if not real:
        lines.append("")
        lines.append("No CAN hardware detected. Replay a log instead: "
                     "--source replay:<log.csv>")
    return "\n".join(lines)


def open_source(spec=None, channel="0", bitrate=500000, can_filters=None,
                speed=1.0, loop=False):
    """Open a CAN source and return a Bus-like object.

    Raises on failure so the caller can report it; never returns None.
    """
    spec = spec or default_source()
    require_python_can()

    if spec.startswith("replay:") or spec.endswith(".csv"):
        path = spec[len("replay:"):] if spec.startswith("replay:") else spec
        if not os.path.exists(path):
            raise FileNotFoundError("Replay log not found: %s" % path)
        return CsvReplayBus(path, speed=speed, loop=loop,
                            can_filters=can_filters)

    kwargs = {"channel": channel, "interface": spec}
    if spec not in BITRATE_SET_ELSEWHERE:
        kwargs["bitrate"] = bitrate
    elif bitrate:
        # SocketCAN takes its bitrate from the network interface, and quietly
        # ignores one passed here - so say so rather than let --bitrate look
        # like it did something.
        print(f"ℹ️  '{spec}' takes its bitrate from the interface, not from "
              f"--bitrate. Set it with:\n"
              f"     sudo ip link set {channel} up type can bitrate {bitrate}")
    bus = CanBus(**kwargs)
    if can_filters:
        bus.set_filters(can_filters)
    return bus
