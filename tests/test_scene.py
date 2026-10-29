# -*- coding: utf-8 -*-
"""
Tests for static / moving classification, the coverage map, CCVS ego speed
reaching the frame, and the controlled range-check comparison.
"""

import csv
import math

import pytest

from can_frame_processor import CANFrameProcessor, decode_ccvs
from can_source import CsvReplayBus
from inject_vehicle_data import decode_ccvs as injector_decode_ccvs
from inject_vehicle_data import encode_ccvs
from range_check import compare
from scene_model import (CoverageMap, SceneModel, classify_point,
                         classify_track)


# ------------------------------------------------------------ classification

def test_stationary_object_seen_from_a_moving_vehicle_is_static():
    # Dead ahead at 10 m/s: a static object closes at exactly ego speed
    assert classify_point(0.0, 20.0, -10.0, 10.0) == (pytest.approx(0.0), False)
    # 60 degrees off boresight it only closes at ego * cos(60)
    lateral, range_m = 20.0 * math.sin(math.radians(60)), 20.0 * math.cos(math.radians(60))
    residual, moving = classify_point(lateral, range_m, -5.0, 10.0)
    assert residual == pytest.approx(0.0, abs=1e-9) and not moving


def test_object_closing_on_a_stationary_radar_is_moving_and_closing():
    residual, moving = classify_point(0.0, 15.0, -2.0, 0.0)
    assert moving and residual < 0


def test_track_velocity_is_judged_over_the_ground():
    # A parked car ahead closes at ego speed: static over the ground
    assert classify_track(0.0, -12.0, 12.0)[1] is False
    # A car pulling away at the same relative rate while we are parked moves
    assert classify_track(0.0, 3.0, 0.0)[1] is True


def test_scene_reads_ego_speed_and_its_source():
    scene = SceneModel()
    scene.update({
        "ego_speed": {"ego_speed": "10.00", "source": "CCVS"},
        "Point_cloud": {"Detect_Points": [{"D_X": 0.0, "D_Y": 20.0, "Doppler": -10.0}]},
        "Tracked_obj": {"Track_Objects": [{"T_X": 1.0, "T_Y": 30.0,
                                           "T_Vx": 0.0, "T_Vy": -2.0}]},
    })
    assert scene.ego_known and scene.ego_source == "CCVS"
    assert scene.counts() == {"moving_points": 0, "static_points": 1,
                              "moving_tracks": 1, "static_tracks": 0}


def test_scene_without_ego_speed_assumes_a_stationary_radar_and_says_so():
    scene = SceneModel()
    scene.update({"Point_cloud": {"Detect_Points": [
        {"D_X": 0.0, "D_Y": 20.0, "Doppler": -10.0}]}})
    assert not scene.ego_known
    assert scene.counts()["moving_points"] == 1


# ------------------------------------------------------------------ coverage

def test_coverage_extent_ignores_a_lone_outlier():
    coverage = CoverageMap()
    for i in range(1000):
        coverage.add(5.0 + (i % 45), -10.0 + (i % 50))   # -10..+39 deg, 5..49 m
    coverage.add(80.0, 85.0)                               # one ghost
    az_lo, az_hi, r_max = coverage.observed_extent()
    assert -12.0 <= az_lo <= -9.0
    assert 39.0 <= az_hi <= 42.0
    assert r_max <= 50.0


def test_coverage_reset_forgets_everything():
    coverage = CoverageMap()
    coverage.add(10.0, 0.0)
    coverage.reset()
    assert coverage.observed_extent() is None and list(coverage.cells()) == []


# ------------------------------------------------------------ CCVS ego speed

def test_processor_decodes_ccvs_exactly_as_the_injector_encodes_it():
    frame = encode_ccvs(13.9)
    assert decode_ccvs(frame) == injector_decode_ccvs(frame)
    assert decode_ccvs(frame) == pytest.approx(13.9, abs=0.01)


def test_ccvs_becomes_the_frame_ego_speed(tmp_path, monkeypatch):
    """Regression: CCVS reached the radar but the visualizer showed no ego
    speed, because only 0x3F5 - which the radar never sends - was decoded."""
    source = tmp_path / "source.csv"
    ccvs = " ".join("%02X" % b for b in encode_ccvs(11.1))
    with open(source, "w", newline="") as handle:
        csv.writer(handle).writerows([
            [1000.00, "18FEF111", 8, ccvs],
            [1000.01, "C1", 8, "02 01 04 03 06 05 08 07"],
            [1000.02, "501", 8, "00 00 00 00 00 00 00 00"],
        ])
    monkeypatch.chdir(tmp_path)
    processor = CANFrameProcessor(csv_filename="raw.csv",
                                  source="replay:%s" % source, replay_speed=0)
    assert processor.setup_can_bus()
    processor.collect_can_messages()
    processor._raw_csv_handle.close()

    result = processor.process_complete_frame(processor.frame_queue.get_nowait())
    assert result["ego_speed"]["source"] == "CCVS"
    assert float(result["ego_speed"]["ego_speed"]) == pytest.approx(11.1, abs=0.01)


# --------------------------------------------------------------- range check

def write_objects(path, frames):
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame_num", "timestamp", "source", "obj_index",
                         "assoc_id", "distX", "distY"])
        for frame, rows in enumerate(frames):
            for source, long_m, lat_m in rows:
                writer.writerow([frame, frame * 0.08, source, 1, "", long_m, lat_m])


def test_range_check_separates_the_object_from_the_bench(tmp_path):
    wall = ("point", 10.2, 0.3)
    empty, placed = tmp_path / "empty.csv", tmp_path / "placed.csv"
    write_objects(empty, [[wall]] * 50)
    # The object shows up 9 m too far, and never where it really is
    write_objects(placed, [[wall, ("point", 9.1, 0.1)]] * 50)

    report, _, _ = compare(str(empty), str(placed), truth_m=1.0)
    points = report["point"]
    assert points["found_at_truth_rate"] == 0.0
    assert len(points["new_cells"]) == 1               # the wall cancels out
    assert points["new_cells"][0]["range_m"] == pytest.approx(9.25)
