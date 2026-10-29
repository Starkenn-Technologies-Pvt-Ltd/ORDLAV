# -*- coding: utf-8 -*-
"""
Decoder regression tests, checked against Starkenn Radar DBC V6.

Every bug found during bring-up lived here: threshold scaling read as whole
metres instead of 0.5 m per LSB, sign extension compared against 32567, ego
speed treated as km/h when the DBC says m/s, and the track record's last four
bytes discarded as spare when the firmware writes object extent into them.
Each has a test below so it cannot come back silently.
"""

import struct

import pytest

from can_frame_processor import CANFrameProcessor
from make_sample_log import (encode_alarm_thresholds, encode_count,
                             encode_ego_speed, encode_hil_target,
                             encode_object, encode_time_msg, encode_yaw)


@pytest.fixture
def decoder():
    """A decoder with no threads, sockets or log files behind it."""
    return CANFrameProcessor.__new__(CANFrameProcessor)


def as_hex(payload):
    return " ".join("%02X" % b for b in payload)


# ---------------------------------------------------------------- bit layout

def test_generic_extractor_matches_dbc_object_layout(decoder):
    """The extractor underpins every signal, so pin it to the known layout."""
    import random
    random.seed(1)
    for _ in range(500):
        data = [random.randrange(256) for _ in range(8)]
        assert decoder._bits(data, 0, 13) == data[0] | ((data[1] & 0x1F) << 8)
        assert decoder._bits(data, 13, 12) == (
            ((data[1] & 0xE0) >> 5) | (data[2] << 3) | ((data[3] & 0x01) << 11))
        assert decoder._bits(data, 25, 11) == (
            ((data[3] & 0xFE) >> 1) | ((data[4] & 0x0F) << 7))
        assert decoder._bits(data, 36, 11) == (
            ((data[4] & 0xF0) >> 4) | ((data[5] & 0x7F) << 4))


# ------------------------------------------------------------- object signal

@pytest.mark.parametrize("rng, lat, v_rng, v_lat", [
    (0.3, 0.0, 0.0, 0.0),        # at the sensor's minimum-range gate
    (7.0, -2.688, 1.5, -0.1),    # a real frame from a bench capture
    (80.0, 2.0, -13.6, 0.4),
    (200.0, -60.0, 25.0, -25.0),  # near the ends of the signal ranges
])
def test_object_signal_round_trip(decoder, rng, lat, v_rng, v_lat):
    decoded = decoder.parse_object_signal(
        as_hex(encode_object(rng, lat, v_rng, v_lat)))
    assert decoded is not None
    x, y, vx, vy = (float(v) for v in decoded)
    # Tolerances are one quantisation step of each signal
    assert x == pytest.approx(rng, abs=0.032)
    assert y == pytest.approx(lat, abs=0.064)
    assert vx == pytest.approx(v_rng, abs=0.1)
    assert vy == pytest.approx(v_lat, abs=0.1)


def test_object_signal_rejects_short_payload(decoder):
    assert decoder.parse_object_signal("01 02 03") is None


# ----------------------------------------------------------------- scalings

def test_alarm_thresholds_are_half_metre_per_lsb(decoder):
    """Regression: raw bytes were returned, so every threshold read double."""
    payload = encode_alarm_thresholds((30, 20, 10), (60, 40, 20))
    assert decoder.parse_0x6F1_Alarm_distance_Msg(as_hex(payload)) == [
        30.0, 20.0, 10.0, 60.0, 40.0, 20.0]
    # and the raw byte really is twice the metre value
    assert payload[0] == 60


def test_alarm_level_is_two_bits(decoder):
    """Regression: the field was masked with 0xFF."""
    assert decoder.parse_0X001_Alarm_Info("FF 00 00 00 00 00 00 00") == 3
    assert decoder.parse_0X001_Alarm_Info("02 00 00 00 00 00 00 00") == 2


def test_ego_speed_is_metres_per_second(decoder):
    """Regression: the value was treated as km/h, a factor of 3.6 out."""
    assert float(decoder.parse_3F5(as_hex(encode_ego_speed(11.1)))) == \
        pytest.approx(11.1, abs=0.01)


def test_yaw_rate_carries_its_sign(decoder):
    assert decoder.parse_yaw(as_hex(encode_yaw(0.6)))["yaw_rate"] == \
        pytest.approx(0.6, abs=0.01)
    assert decoder.parse_yaw(as_hex(encode_yaw(-0.6)))["yaw_rate"] == \
        pytest.approx(-0.6, abs=0.01)


def test_time_message(decoder):
    decoded = decoder.parse_time_msg(as_hex(encode_time_msg(3.27, 1.5, 1)))
    assert decoded["ttc"] == pytest.approx(3.27, abs=0.01)
    assert decoded["pet"] == pytest.approx(1.5, abs=0.01)
    assert decoded["relevant_object"] == 1


def test_object_count_carries_cycle_counter(decoder):
    decoded = decoder.parse_object_count(as_hex(encode_count(3, 50, 1234)))
    assert (decoded["count"], decoded["cycle_time"], decoded["cycle_count"]) \
        == (3, 50, 1234)


# ------------------------------------------------------------- HIL ground truth

def test_hil_target_classification(decoder):
    payload = encode_hil_target(50.0, 1.0, -10.0, 0.0,
                                type_code=3, trajectory=1, rcs=22)
    decoded = decoder.parse_hil_target(as_hex(payload))
    assert decoded["type"] == "car"
    assert decoded["trajectory"] == "oncoming"
    assert decoded["rcs_dbsm"] == 22
    assert float(decoded["x_val"]) == pytest.approx(50.0, abs=0.032)


# --------------------------------------------------------------- raw counts

def test_raw_counts_reproduce_the_metre_value(decoder):
    """The on-screen raw readout must match the decoded distance exactly."""
    payload = encode_object(22.69, 0.0, 0.0, 0.0)
    raw = decoder.object_raw_counts(as_hex(payload))
    x_val = float(decoder.parse_object_signal(as_hex(payload))[0])
    assert 0.032 * raw["x_raw"] - 60.8 == pytest.approx(x_val, abs=1e-9)
