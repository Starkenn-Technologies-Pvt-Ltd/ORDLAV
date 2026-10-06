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

# A gap longer than this in the log is collapsed, so a recording with a long
# idle stretch does not stall the replay.
MAX_REPLAY_GAP_S = 0.5

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
                    can.Message(
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


def open_source(spec=None, channel="0", bitrate=500000, can_filters=None,
                speed=1.0, loop=False):
    """Open a CAN source and return a Bus-like object.

    Raises on failure so the caller can report it; never returns None.
    """
    spec = spec or default_source()

    if spec.startswith("replay:") or spec.endswith(".csv"):
        path = spec[len("replay:"):] if spec.startswith("replay:") else spec
        if not os.path.exists(path):
            raise FileNotFoundError("Replay log not found: %s" % path)
        return CsvReplayBus(path, speed=speed, loop=loop,
                            can_filters=can_filters)

    kwargs = {"channel": channel, "interface": spec}
    if spec != "virtual":
        kwargs["bitrate"] = bitrate
    bus = can.interface.Bus(**kwargs)
    if can_filters:
        bus.set_filters(can_filters)
    return bus
