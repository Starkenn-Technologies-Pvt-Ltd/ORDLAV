# -*- coding: utf-8 -*-
"""
Scene model behind the visualizer: static / moving classification and the
area the radar has actually covered.

Kept free of Qt so the logic can be tested without a display.

Coordinates follow the processor's frame JSON: D_X / T_X are lateral (m,
positive to the right), D_Y / T_Y are range along boresight (m). Point Doppler
is radial velocity relative to the radar, negative when closing. Track
velocity (T_Vx lateral, T_Vy range) is relative to the radar as well.

Static vs moving needs ego speed. A stationary object seen from a vehicle
moving forward at v closes at v * cos(azimuth), so its Doppler is
-v * cos(azimuth); what is left after removing that is the object's own
motion. With no ego speed the radar is assumed stationary - right on a bench,
wrong in a vehicle - and the model says so via ego_known.
"""

import math
from collections import deque

# Residual speed above which a detection counts as moving. One Doppler bin on
# the bench unit is about 0.13 m/s, so this sits well clear of quantisation.
MOVING_THRESHOLD_MPS = 0.6

# Frames of point history kept for the range-Doppler / range-azimuth panels.
# The radar sends ~4 points a frame, too sparse to read one frame at a time.
HISTORY_FRAMES = 30


def azimuth_deg(lateral, range_m):
    """Azimuth from boresight, positive to the right."""
    return math.degrees(math.atan2(lateral, range_m))


def classify_point(lateral, range_m, doppler, ego_mps):
    """(residual_mps, moving) for one detection.

    residual is the Doppler left after removing what a stationary object
    would show; its sign keeps the closing (<0) / opening (>0) sense.
    """
    expected = -ego_mps * math.cos(math.atan2(lateral, range_m))
    residual = doppler - expected
    return residual, abs(residual) > MOVING_THRESHOLD_MPS


def classify_track(v_lateral, v_range, ego_mps):
    """(ground_speed_mps, moving) for one track, from its relative velocity."""
    ground = math.hypot(v_lateral, v_range + ego_mps)
    return ground, ground > MOVING_THRESHOLD_MPS


class CoverageMap:
    """Polar occupancy grid: how often each range/azimuth cell had a detection.

    Built up over time, it shows the area the radar really sees on this
    mounting, as opposed to its datasheet field of view.
    """

    def __init__(self, max_range=120.0, range_step=1.0,
                 az_limit=90.0, az_step=3.0):
        self.max_range = max_range
        self.range_step = range_step
        self.az_limit = az_limit
        self.az_step = az_step
        self.n_range = int(math.ceil(max_range / range_step))
        self.n_az = int(round(2 * az_limit / az_step))
        self.reset()

    def reset(self):
        self.counts = [[0] * self.n_az for _ in range(self.n_range)]
        self.total = 0
        self.peak = 0

    def add(self, range_m, az_deg):
        if not (0.0 <= range_m < self.max_range and -self.az_limit <= az_deg < self.az_limit):
            return
        ri = int(range_m / self.range_step)
        ai = int((az_deg + self.az_limit) / self.az_step)
        self.counts[ri][ai] += 1
        self.total += 1
        self.peak = max(self.peak, self.counts[ri][ai])

    def cells(self):
        """Yield (range_lo, range_hi, az_lo, az_hi, count) for occupied cells."""
        for ri, row in enumerate(self.counts):
            for ai, count in enumerate(row):
                if count:
                    yield (ri * self.range_step, (ri + 1) * self.range_step,
                           ai * self.az_step - self.az_limit,
                           (ai + 1) * self.az_step - self.az_limit, count)

    def observed_extent(self, trim=0.01):
        """(az_min, az_max, range_max) covering all but `trim` of detections
        at each edge, or None before anything has been seen. Trimming keeps a
        single multipath ghost from defining the field of view."""
        if not self.total:
            return None
        az_marginal = [sum(row[ai] for row in self.counts) for ai in range(self.n_az)]
        range_marginal = [sum(row) for row in self.counts]
        cut = self.total * trim

        def first_over(marginal):
            running = 0
            for index, count in enumerate(marginal):
                running += count
                if running > cut:
                    return index
            return len(marginal) - 1

        az_lo = first_over(az_marginal)
        az_hi = len(az_marginal) - 1 - first_over(az_marginal[::-1])
        r_hi = len(range_marginal) - 1 - first_over(range_marginal[::-1])
        return (az_lo * self.az_step - self.az_limit,
                (az_hi + 1) * self.az_step - self.az_limit,
                (r_hi + 1) * self.range_step)


class SceneModel:
    """Classified view of the latest frame, plus history and coverage."""

    def __init__(self, history_frames=HISTORY_FRAMES, coverage=None):
        self.coverage = coverage or CoverageMap()
        self.history = deque(maxlen=history_frames)
        self.points = []
        self.tracks = []
        self.ego_mps = 0.0
        self.ego_known = False
        self.ego_source = None

    def reset_coverage(self):
        self.coverage.reset()

    def update(self, frame):
        frame = frame or {}
        ego = frame.get('ego_speed') or {}
        try:
            self.ego_mps = float(ego.get('ego_speed'))
            self.ego_known = True
            self.ego_source = ego.get('source')
        except (TypeError, ValueError):
            self.ego_mps, self.ego_known, self.ego_source = 0.0, False, None

        raw_points = ((frame.get('Point_cloud') or {}).get('Detect_Points') or [])
        raw_tracks = ((frame.get('Tracked_obj') or {}).get('Track_Objects') or [])

        self.points = []
        for point in raw_points:
            try:
                lateral = float(point.get('D_X', 0.0))
                range_m = float(point.get('D_Y', 0.0))
                doppler = float(point.get('Doppler', 0.0) or 0.0)
            except (TypeError, ValueError):
                continue
            residual, moving = classify_point(lateral, range_m, doppler, self.ego_mps)
            radial = math.hypot(lateral, range_m)
            az = azimuth_deg(lateral, range_m)
            self.points.append({
                'lateral': lateral, 'range': range_m, 'radial': radial,
                'azimuth': az, 'doppler': doppler, 'residual': residual,
                'moving': moving, 'peak': point.get('Peak'),
            })
            self.coverage.add(radial, az)

        self.tracks = []
        for track in raw_tracks:
            try:
                lateral = float(track.get('T_X', 0.0))
                range_m = float(track.get('T_Y', 0.0))
                v_lat = float(track.get('T_Vx', 0.0) or 0.0)
                v_rng = float(track.get('T_Vy', 0.0) or 0.0)
            except (TypeError, ValueError):
                continue
            ground, moving = classify_track(v_lat, v_rng, self.ego_mps)
            self.tracks.append({
                'lateral': lateral, 'range': range_m,
                'radial': math.hypot(lateral, range_m),
                'azimuth': azimuth_deg(lateral, range_m),
                'v_lateral': v_lat, 'v_range': v_rng,
                'ground_speed': ground, 'moving': moving,
                'id': track.get('T_ID'),
                'size_x': float(track.get('T_SX', 0.0) or 0.0),
                'size_y': float(track.get('T_SY', 0.0) or 0.0),
            })

        self.history.append(self.points)

    def counts(self):
        return {
            'moving_points': sum(p['moving'] for p in self.points),
            'static_points': sum(not p['moving'] for p in self.points),
            'moving_tracks': sum(t['moving'] for t in self.tracks),
            'static_tracks': sum(not t['moving'] for t in self.tracks),
        }
