# -*- coding: utf-8 -*-
"""
Frame-to-frame association, to give radar objects a stable identity.

The radar transmits no object ID in any documented message, so there is no
vendor identity to follow. This assigns one: each frame's objects are matched
to the previous frame's by predicted position, and a matched object keeps its
number.

What that does and does not tell you:

  * It does measure detection stability - whether one physical target stays a
    single, continuously reported object, and where it splits, merges or drops
    out. That is the question the function-performance work actually asks.
  * It does NOT verify the radar's own ID consistency. The radar has no ID to
    be consistent about. An ID here is ours, not the sensor's.

Association is greedy nearest-neighbour within a gate, over positions
predicted from the previous velocity. At 20 Hz a target closing at 14 m/s
moves 0.7 m between frames, well inside the default gate, so the simple
approach is sufficient; swapping in a global-optimal assignment would change
little.
"""

import math


class TrackRecord:
    """Book-keeping for one assigned identity."""

    __slots__ = ("id", "dist_x", "dist_y", "vel_x", "vel_y",
                 "first_frame", "last_frame", "frames_seen", "coasted")

    def __init__(self, track_id, frame, dist_x, dist_y, vel_x, vel_y):
        self.id = track_id
        self.dist_x = dist_x
        self.dist_y = dist_y
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.first_frame = frame
        self.last_frame = frame
        self.frames_seen = 1
        self.coasted = 0

    def predict(self, dt):
        """Where this object should be after dt seconds."""
        return (self.dist_x + self.vel_x * dt,
                self.dist_y + self.vel_y * dt)

    def update(self, frame, dist_x, dist_y, vel_x, vel_y):
        self.dist_x, self.dist_y = dist_x, dist_y
        self.vel_x, self.vel_y = vel_x, vel_y
        self.last_frame = frame
        self.frames_seen += 1
        self.coasted = 0


class ObjectTracker:
    """Assigns stable ids to per-frame object detections.

    Args:
        gate_m: furthest a detection may sit from a prediction and still be
            considered the same object.
        max_coast_frames: how long an unmatched identity is held before it is
            retired, so a one-frame dropout does not burn a new id.
    """

    def __init__(self, gate_m=3.5, max_coast_frames=5):
        self.gate_m = gate_m
        self.max_coast_frames = max_coast_frames
        self._next_id = 1
        self._active = []
        self._last_timestamp = None
        self.history = {}   # id -> dict, for the continuity report

    def update(self, detections, frame=0, timestamp=None):
        """Associate one frame of detections.

        Args:
            detections: sequence of (dist_x, dist_y, vel_x, vel_y), vehicle
                frame - dist_x longitudinal, dist_y lateral.
        Returns:
            A list of ids, one per detection, in the order given.
        """
        dt = 0.05
        if timestamp is not None and self._last_timestamp is not None:
            delta = timestamp - self._last_timestamp
            if 0 < delta < 1.0:
                dt = delta
        if timestamp is not None:
            self._last_timestamp = timestamp

        predictions = [(record, record.predict(dt)) for record in self._active]

        # Every detection/identity pair inside the gate, nearest first.
        pairs = []
        for d_index, (dist_x, dist_y, _, _) in enumerate(detections):
            for t_index, (_, (pred_x, pred_y)) in enumerate(predictions):
                distance = math.hypot(dist_x - pred_x, dist_y - pred_y)
                if distance <= self.gate_m:
                    pairs.append((distance, d_index, t_index))
        pairs.sort()

        assigned = [None] * len(detections)
        used_tracks = set()
        for _, d_index, t_index in pairs:
            if assigned[d_index] is not None or t_index in used_tracks:
                continue
            record = predictions[t_index][0]
            dist_x, dist_y, vel_x, vel_y = detections[d_index]
            record.update(frame, dist_x, dist_y, vel_x, vel_y)
            assigned[d_index] = record.id
            used_tracks.add(t_index)

        # Anything left over is something we have not seen before.
        for d_index, track_id in enumerate(assigned):
            if track_id is not None:
                continue
            dist_x, dist_y, vel_x, vel_y = detections[d_index]
            record = TrackRecord(self._next_id, frame, dist_x, dist_y, vel_x, vel_y)
            self._next_id += 1
            self._active.append(record)
            assigned[d_index] = record.id

        # Retire identities that have gone unmatched for too long.
        survivors = []
        for t_index, (record, _) in enumerate(predictions):
            if t_index in used_tracks:
                survivors.append(record)
                continue
            record.coasted += 1
            if record.coasted <= self.max_coast_frames:
                record.dist_x, record.dist_y = record.predict(dt)
                survivors.append(record)
        self._active = survivors + [r for r in self._active
                                    if r not in [p[0] for p in predictions]]

        for record in self._active:
            entry = self.history.setdefault(record.id, {})
            entry.update(first_frame=record.first_frame,
                         last_frame=record.last_frame,
                         frames_seen=record.frames_seen)

        return assigned

    @property
    def active_ids(self):
        return [record.id for record in self._active if record.coasted == 0]
