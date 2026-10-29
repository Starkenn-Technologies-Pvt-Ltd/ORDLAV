# -*- coding: utf-8 -*-
"""
Tests for frame decoding, identity association, replay and bus monitoring.

These cover the behaviours that were verified by hand during bring-up: the
mmWave TLV walk (including the truncated and delimiter-only frames that used
to raise), stable identities across dropouts, replay timing and loop handling,
and the bus-rate runaway a looping replay used to cause.
"""

import csv
import time

import pytest

from can_frame_processor import BusMonitor, CANFrameProcessor
from can_source import CsvReplayBus
from make_sample_log import (FRAME_DELIMITER, build_radar_payload,
                             chunk_to_messages, encode_object)
from object_tracker import ObjectTracker


@pytest.fixture
def decoder():
    return CANFrameProcessor.__new__(CANFrameProcessor)


def radar_hex(points, clusters, tracks, frame_num=1):
    """The hex buffer the processor assembles from the C1/D1/D2/D3 stream."""
    header, tlvs = build_radar_payload(frame_num, points, clusters, tracks)
    payload = FRAME_DELIMITER + header + tlvs
    return " ".join("%02X" % b for b in payload)


# ------------------------------------------------------------------- TLV walk

def test_frame_decodes_points_clusters_and_tracks(decoder):
    points = [(1.0, 20.0, -5.0), (-2.0, 30.0, 2.0)]
    clusters = [(1.0, 20.0, 1.8, 4.5)]
    tracks = [(1.0, 20.0, 0.2, -5.0)]
    detected, tracked, clustered = decoder.parse_frame_data(
        radar_hex(points, clusters, tracks))

    assert len(detected) == 2
    assert len(clustered) == 1
    assert len(tracked) == 1
    assert tracked[0]["T_Y"] == pytest.approx(20.0, abs=0.01)
    # Extent comes from the last four bytes of the track record
    assert tracked[0]["T_SX"] == pytest.approx(1.8, abs=0.01)
    assert tracked[0]["T_SY"] == pytest.approx(4.5, abs=0.01)


def test_delimiter_only_frame_decodes_to_nothing(decoder):
    """The end of every recording looks like this; it used to raise."""
    only_delimiter = " ".join("%02X" % b for b in FRAME_DELIMITER)
    assert decoder.parse_frame_data(only_delimiter) == ([], [], [])


def test_truncated_frame_keeps_what_it_decoded(decoder):
    full = radar_hex([(0.0, 10.0, 0.0)] * 5, [], [])
    tokens = full.split()
    truncated = " ".join(tokens[:len(tokens) // 2])
    detected, tracked, clustered = decoder.parse_frame_data(truncated)
    assert 0 < len(detected) < 5          # some survived, none invented
    assert (tracked, clustered) == ([], [])


def test_tracks_below_the_minimum_range_are_skipped_not_fatal(decoder):
    """A padding slot must not discard the real tracks behind it."""
    tracks = [(0.0, 0.1, 0.0, 0.0), (1.0, 25.0, 0.0, -5.0)]
    _, tracked, _ = decoder.parse_frame_data(radar_hex([], [], tracks))
    assert len(tracked) == 1
    assert tracked[0]["T_Y"] == pytest.approx(25.0, abs=0.01)


# ------------------------------------------------------------------ identity

def test_identity_is_stable_and_survives_a_dropout():
    tracker = ObjectTracker()
    assigned = []
    for frame in range(12):
        detections = [] if frame == 5 else [(50 - frame * 0.7, 0.0, -14.0, 0.0)]
        assigned.append(tracker.update(detections, frame=frame,
                                       timestamp=frame * 0.05))
    seen = [row[0] for row in assigned if row]
    assert set(seen) == {1}, "a one-frame dropout must not burn a new identity"


def test_a_second_object_gets_its_own_identity():
    tracker = ObjectTracker()
    for frame in range(40):
        detections = [(80 - frame * 0.7, 0.1, -14.0, 0.0)]
        if frame >= 20:
            detections.append((100 - frame * 0.3, -3.5, -6.0, 0.0))
        ids = tracker.update(detections, frame=frame, timestamp=frame * 0.05)
    assert ids == [1, 2]


def test_identities_follow_objects_through_a_slot_swap():
    """CAN slots are ranked by proximity, so they reassign; identities must not."""
    tracker = ObjectTracker()
    # B starts further away and overtakes A, so their slot order flips
    for frame in range(30):
        a = 40.0 - frame * 0.2
        b = 50.0 - frame * 1.0
        detections = sorted([(a, 0.0, -4.0, 0.0), (b, 1.0, -20.0, 0.0)])
        ids = tracker.update(detections, frame=frame, timestamp=frame * 0.05)
    # Both originals are still present after the ranking changed
    assert sorted(ids) == [1, 2]


# -------------------------------------------------------------------- replay

def write_log(path, rows):
    with open(path, "w", newline="") as handle:
        csv.writer(handle).writerows(rows)


def test_replay_filters_to_the_requested_ids(tmp_path):
    log = tmp_path / "log.csv"
    write_log(log, [
        [1000.0, "510", 8, " ".join("%02X" % b for b in encode_object(10, 0, 0, 0))],
        [1000.01, "7FF", 8, "00 00 00 00 00 00 00 00"],
    ])
    bus = CsvReplayBus(str(log), speed=0,
                       can_filters=[{"can_id": 0x510, "can_mask": 0x7FF}])
    ids = []
    while True:
        message = bus.recv(timeout=0.01)
        if message is None:
            break
        ids.append(message.arbitration_id)
    assert ids == [0x510]


def test_replay_reproduces_the_recorded_cadence(tmp_path):
    log = tmp_path / "log.csv"
    write_log(log, [[1000.0 + i * 0.01, "510", 8,
                     "00 00 00 00 00 00 00 00"] for i in range(50)])
    bus = CsvReplayBus(str(log), speed=1.0)
    started = time.monotonic()
    # A small poll timeout: once the log is spent, recv sleeps for the whole
    # timeout before reporting exhaustion, which would otherwise dominate.
    while not bus.exhausted:
        bus.recv(timeout=0.02)
    elapsed = time.monotonic() - started
    assert 0.40 < elapsed < 0.70, f"0.49 s of log replayed in {elapsed:.2f} s"


def test_replay_reports_exhaustion_once_finished(tmp_path):
    log = tmp_path / "log.csv"
    write_log(log, [[1000.0, "510", 8, "00 00 00 00 00 00 00 00"]])
    bus = CsvReplayBus(str(log), speed=0)
    assert bus.recv(timeout=0.01) is not None
    assert bus.recv(timeout=0.01) is None
    assert bus.exhausted


# --------------------------------------------------------------- bus monitor

def test_bus_monitor_reports_per_message_rate_not_their_sum():
    monitor = BusMonitor()
    # Four messages in one group, each at 20 Hz
    for step in range(100):
        for can_id in (0x100, 0x101, 0x102, 0x103):
            monitor.record(can_id, 1000.0 + step * 0.05)
    assert monitor.snapshot()["HIL truth"]["hz"] == pytest.approx(20, abs=1.5)


def test_bus_monitor_survives_a_looping_replay():
    """Regression: the clock jumping back made the rates run away."""
    monitor = BusMonitor()
    for step in range(100):
        monitor.record(0x510, 1000.0 + step * 0.05)
    for step in range(100):            # the log restarts
        monitor.record(0x510, 1000.0 + step * 0.05)
    assert monitor.snapshot()["objects"]["hz"] == pytest.approx(20, abs=2)


def test_bus_monitor_distinguishes_never_sent_from_silent():
    monitor = BusMonitor()
    for step in range(20):
        monitor.record(0x510, 1000.0 + step * 0.05)
    monitor.record(0x6F1, 1000.0)          # once, long ago
    monitor.record(0x510, 1010.0)          # now
    snapshot = monitor.snapshot()
    assert snapshot["objects"]["state"] == "live"
    assert snapshot["alarm"]["state"] == "silent"
    assert snapshot["HIL truth"]["state"] == "never"
