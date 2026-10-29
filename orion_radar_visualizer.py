# -*- coding: utf-8 -*-
"""
Orion Radar Visualizer — live bird's-eye view of the radar frame stream.

Connects to CAN_Frame_Processor_v1.4.py on localhost:5001 and renders each
frame: detected point cloud, clusters, tracked objects, the static and moving
objects of interest, alarm threshold bands and ego speed.

The scene is painted in one pass with QPainter rather than by moving child
widgets around, which keeps the frame cost flat and allows real geometry —
velocity vectors, cluster extents, threshold bands.
"""

import argparse
import datetime
import json
import math
import socket
import sys
import time
from collections import deque

from PyQt5.QtCore import (QPointF, QRectF, QSettings, Qt, QThread, QTimer,
                          pyqtSignal)
from PyQt5.QtGui import (QBrush, QColor, QFont, QFontMetrics, QPainter,
                         QPainterPath, QPen, QPixmap, QPolygonF)
from PyQt5.QtWidgets import (QApplication, QFrame, QHBoxLayout, QHeaderView,
                             QLabel, QMainWindow, QPushButton, QSizePolicy,
                             QTableWidget, QTableWidgetItem, QVBoxLayout,
                             QWidget)

from scene_model import SceneModel, classify_track

HOST = '127.0.0.1'   # CAN Processor IP
PORT = 5001          # CAN Processor Port

# ── Themes ─────────────────────────────────────────────────────────────────
# Each theme is a flat map of role -> hex colour. Roles are referenced by name
# everywhere, so adding a theme needs no changes to the drawing code.
THEMES = {
    'dark': {
        'bg_deep': '#080B10', 'bg_plot': '#0B0F16',
        'grid_minor': '#161C26', 'grid_major': '#222C3A', 'axis_line': '#35435A',
        'text_dim': '#6A7A92', 'text_mid': '#9AAAC2', 'text_bright': '#E6EDF7',
        'header_bg': '#0C1119', 'panel_bg': '#0E131B', 'panel_border': '#1E2735',
        'label_bg': '#080B10',
        'point_near': '#FF8A3D', 'point_still': '#36D6E7', 'point_away': '#4D7CFF',
        'cluster': '#FFC53D', 'track': '#5AA9FF',
        'static_ooi': '#FF4D6D', 'dynamic_ooi': '#FF2D55', 'ego': '#2BD97C',
        'alarm_1': '#2BD97C', 'alarm_2': '#FFB020', 'alarm_3': '#FF3B30',
        'marker_edge': '#FFFFFF',
        'coverage': '#2EC4A6', 'fov': '#8A9BB5', 'track_static': '#7D8BA3',
    },
    'light': {
        'bg_deep': '#EDF0F5', 'bg_plot': '#FFFFFF',
        'grid_minor': '#E4E9F1', 'grid_major': '#CCD5E2', 'axis_line': '#93A1B8',
        'text_dim': '#78849A', 'text_mid': '#4B5870', 'text_bright': '#121923',
        'header_bg': '#FFFFFF', 'panel_bg': '#FFFFFF', 'panel_border': '#D5DCE8',
        'label_bg': '#FFFFFF',
        'point_near': '#D9480F', 'point_still': '#0B7F93', 'point_away': '#2A52CC',
        'cluster': '#B8860B', 'track': '#1760C4',
        'static_ooi': '#C81E43', 'dynamic_ooi': '#D00A3C', 'ego': '#0E8A4C',
        'alarm_1': '#15803D', 'alarm_2': '#C2740A', 'alarm_3': '#DC2626',
        'marker_edge': '#1B2430',
        'coverage': '#0E8F78', 'fov': '#5B6B85', 'track_static': '#6B778C',
    },
}

DEFAULT_THEME = 'dark'

# Opacity of the plate drawn behind on-plot text, per theme
LABEL_BG_ALPHA = {'dark': 205, 'light': 225}


def theme_colour(theme_name, role, alpha=None):
    """QColor for a role in a theme, optionally with an alpha override."""
    colour = QColor(THEMES[theme_name][role])
    if alpha is not None:
        colour.setAlpha(alpha)
    return colour


def rgba(theme_name, role, alpha):
    """CSS rgba() string for the Qt style sheet; alpha is 0-1."""
    colour = QColor(THEMES[theme_name][role])
    return f"rgba({colour.red()},{colour.green()},{colour.blue()},{alpha})"


ALARM_ROLES = ['alarm_1', 'alarm_2', 'alarm_3']
ALARM_LABELS = ['L1', 'L2', 'L3']

# Field of view drawn as the "nominal" fan. These are assumptions, not values
# read from the radar: confirm them against the antenna and chirp design. The
# coverage map beside it shows what the radar actually detected.
NOMINAL_FOV_DEG = 60.0       # each side of boresight
NOMINAL_RANGE_M = 120.0      # mrr_config_chirp_design_MRR120.h


def class_role(item):
    """Theme role for a classified point: closing / static / opening."""
    if not item['moving']:
        return 'point_still'
    return 'point_near' if item['residual'] < 0 else 'point_away'


def nice_ceiling(value, steps=(1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10)):
    """Smallest 'round' number >= value, for axis limits."""
    if value <= 0:
        return 1.0
    magnitude = 10 ** math.floor(math.log10(value))
    for step in steps:
        if step * magnitude >= value:
            return step * magnitude
    return 10 * magnitude


class SocketClientThread(QThread):
    """Background reader for the processor's newline-delimited JSON stream."""

    data_received = pyqtSignal(object)
    connection_changed = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.running = True
        self.buffer = ""

    def run(self):
        while self.running:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                    client_socket.settimeout(5.0)
                    client_socket.connect((HOST, PORT))
                    self.connection_changed.emit(True)
                    self.buffer = ""

                    while self.running:
                        try:
                            chunk = client_socket.recv(65536).decode('utf-8', 'replace')
                        except socket.timeout:
                            continue
                        if not chunk:
                            break  # connection closed

                        self.buffer += chunk
                        while "\n" in self.buffer:
                            message, self.buffer = self.buffer.split("\n", 1)
                            self._handle_message(message.strip())

                self.connection_changed.emit(False)
            except (ConnectionRefusedError, socket.timeout, OSError):
                self.connection_changed.emit(False)
                self.sleep(2)
            except Exception as e:
                print(f"Socket error: {e}")
                self.connection_changed.emit(False)
                self.sleep(2)

    def _handle_message(self, message):
        if not message or not message.startswith(('{', '[')):
            return
        try:
            payload = json.loads(message)
        except json.JSONDecodeError:
            return  # a partial frame from a dropped connection
        if isinstance(payload, dict) and '_keepalive' in payload:
            return
        self.data_received.emit(payload)

    def stop(self):
        self.running = False
        self.quit()
        self.wait(2000)


class RadarPlotWidget(QWidget):
    """Bird's-eye radar scene, painted in a single pass per frame."""

    MARGIN_LEFT = 54
    MARGIN_RIGHT = 16
    MARGIN_TOP = 34
    MARGIN_BOTTOM = 34

    def __init__(self, theme=DEFAULT_THEME, parent=None):
        super(RadarPlotWidget, self).__init__(parent)
        self.setMinimumSize(640, 480)
        self.setAutoFillBackground(True)
        self.theme = theme

        # Scene extent in metres. Only the range is set: the lateral extent
        # follows from the widget's aspect ratio so that both axes share one
        # metres-per-pixel scale. Without that, a rotated footprint shears and
        # headings read as the wrong angle.
        # The bench unit's detections stop near 60 m; scroll to zoom out.
        self.max_range = 80.0
        self.grid_step = 10.0

        self.frame = {}
        self.detected_points = []
        self.tracks = []
        self.clusters = []
        self.static_obj = None
        self.dynamic_obj = None
        self.alarm_thresholds = None
        self.alarm_type = None
        self.ego_speed_mps = None
        self.object_counts = {}
        # Classified points/tracks and coverage, owned by the main window
        self.scene = None

        self._grid_cache = None
        self._grid_key = None

    def c(self, role, alpha=None):
        """Colour for a role in the active theme."""
        return theme_colour(self.theme, role, alpha)

    def set_theme(self, theme):
        self.theme = theme
        self._grid_cache = None
        self.update()


    # ── Data in ────────────────────────────────────────────────────────────

    def update_data(self, json_data):
        """Store the latest frame and request a repaint."""
        try:
            self.frame = json_data or {}

            self.detected_points = self._section(['Point_cloud', 'Detect_Points'])
            self.tracks = self._section(['Tracked_obj', 'Track_Objects'])
            self.clusters = self._section(['Cluster_obj', 'Clusters'])

            self.static_obj = self._object_of_interest('Static_obj')
            self.dynamic_obj = self._object_of_interest('Dynamic_obj')

            thresholds = self.frame.get('Alarm_thresh') or {}
            if thresholds:
                self.alarm_thresholds = thresholds
            status = self.frame.get('Alarm_status') or {}
            self.alarm_type = status.get('Alarm_type')
            self.object_counts = self.frame.get('Object_counts') or {}

            try:
                # DBC Vehicle_Data_Speed is m/s, not km/h
                self.ego_speed_mps = float(
                    (self.frame.get('ego_speed') or {}).get('ego_speed'))
            except (TypeError, ValueError):
                pass  # keep the last known ego speed

            self.update()
        except Exception as e:
            print(f"Error updating radar plot: {e}")

    def _section(self, path):
        node = self.frame
        for key in path:
            if not isinstance(node, dict):
                return []
            node = node.get(key)
        return node if isinstance(node, list) else []

    def _object_of_interest(self, key):
        """CAN 'x' is longitudinal range and 'y' is lateral offset."""
        payload = self.frame.get(key) or {}
        if 'x_val' not in payload or 'y_val' not in payload:
            return None
        try:
            return {
                'range': float(payload['x_val']),
                'lateral': float(payload['y_val']),
                'vx': float(payload.get('Vx_val', 0.0) or 0.0),
                'vy': float(payload.get('Vy_val', 0.0) or 0.0),
            }
        except (TypeError, ValueError):
            return None

    # ── Coordinate mapping ─────────────────────────────────────────────────

    def plot_rect(self):
        return QRectF(
            self.MARGIN_LEFT, self.MARGIN_TOP,
            max(1, self.width() - self.MARGIN_LEFT - self.MARGIN_RIGHT),
            max(1, self.height() - self.MARGIN_TOP - self.MARGIN_BOTTOM),
        )

    def scale(self):
        """Pixels per metre, identical on both axes."""
        return self.plot_rect().height() / self.max_range

    @property
    def half_width(self):
        """Lateral extent visible each side of boresight, in metres."""
        return (self.plot_rect().width() / 2.0) / self.scale()

    def to_pixel_x(self, lateral_m):
        rect = self.plot_rect()
        return rect.center().x() + lateral_m * self.scale()

    def to_pixel_y(self, range_m):
        return self.plot_rect().bottom() - range_m * self.scale()

    def metres_to_pixels_x(self, metres):
        return metres * self.scale()

    def metres_to_pixels_y(self, metres):
        return metres * self.scale()

    # ── Background ─────────────────────────────────────────────────────────

    def _build_grid(self):
        """Grid and axes, cached until the geometry or scale changes."""
        pixmap = QPixmap(self.size())
        pixmap.fill(self.c('bg_deep'))
        rect = self.plot_rect()

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing, False)
        painter.fillRect(rect, self.c('bg_plot'))

        label_font = QFont("Helvetica Neue", 9)
        painter.setFont(label_font)

        # Lateral grid, stepping outward from boresight to the visible edge
        step = self.grid_step
        limit = self.half_width
        ticks = [0.0]
        value = step
        while value <= limit + 1e-6:
            ticks.extend((-value, value))
            value += step

        for lateral in sorted(ticks):
            x = self.to_pixel_x(lateral)
            is_major = abs(lateral) < 1e-6
            painter.setPen(QPen(self.c('axis_line') if is_major else self.c('grid_minor'),
                                2 if is_major else 1))
            painter.drawLine(int(x), int(rect.top()), int(x), int(rect.bottom()))
            painter.setPen(QPen(self.c('text_mid') if is_major else self.c('text_dim')))
            painter.drawText(QRectF(x - 24, rect.bottom() + 6, 48, 16),
                             Qt.AlignCenter, f"{lateral:g}")

        # Range grid, with the top of the scene always labelled
        range_m = 0.0
        range_ticks = []
        while range_m <= self.max_range + 1e-6:
            range_ticks.append(range_m)
            range_m += self.grid_step
        if range_ticks and abs(range_ticks[-1] - self.max_range) > self.grid_step * 0.3:
            range_ticks.append(self.max_range)

        for range_m in range_ticks:
            y = self.to_pixel_y(range_m)
            is_major = abs(range_m % (self.grid_step * 5)) < 1e-6
            painter.setPen(QPen(self.c('grid_major') if is_major else self.c('grid_minor'), 1))
            painter.drawLine(int(rect.left()), int(y), int(rect.right()), int(y))
            if range_m > 0:
                painter.setPen(QPen(self.c('text_mid') if is_major else self.c('text_dim')))
                painter.drawText(QRectF(2, y - 9, self.MARGIN_LEFT - 8, 18),
                                 Qt.AlignRight | Qt.AlignVCenter, f"{range_m:g}")

        # Plot border
        painter.setPen(QPen(self.c('panel_border'), 1))
        painter.drawRect(rect)

        # Axis captions
        painter.setPen(QPen(self.c('text_mid')))
        painter.setFont(QFont("Helvetica Neue", 9, QFont.DemiBold))
        painter.drawText(QRectF(2, 4, 110, 14),
                         Qt.AlignLeft | Qt.AlignVCenter, "RANGE (m)")
        painter.drawText(QRectF(rect.right() - 130, rect.bottom() + 16, 130, 16),
                         Qt.AlignRight | Qt.AlignVCenter, "LATERAL (m)")
        painter.end()
        return pixmap

    # ── Painting ───────────────────────────────────────────────────────────

    def paintEvent(self, event):
        key = (self.width(), self.height(), self.max_range,
               self.grid_step, self.theme)
        if self._grid_cache is None or self._grid_key != key:
            self._grid_cache = self._build_grid()
            self._grid_key = key

        painter = QPainter(self)
        painter.drawPixmap(0, 0, self._grid_cache)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setClipRect(self.plot_rect())

        self._draw_coverage(painter)
        self._draw_fov(painter)
        self._draw_alarm_bands(painter)
        self._draw_boresight(painter)
        self._draw_clusters(painter)
        self._draw_points(painter)
        self._draw_tracks(painter)
        self._draw_objects_of_interest(painter)

        painter.setClipping(False)
        self._draw_ego(painter)
        self._draw_legend(painter)
        painter.end()

    def _draw_alarm_bands(self, painter):
        """Shade the moving-object alarm distance bands along the range axis."""
        if not self.alarm_thresholds:
            return
        rect = self.plot_rect()
        levels = []
        for index, key in enumerate(('Level_1_Mov', 'Level_2_Mov', 'Level_3_Mov')):
            try:
                value = float(self.alarm_thresholds.get(key))
            except (TypeError, ValueError):
                continue
            if value > 0:
                levels.append((value, self.c(ALARM_ROLES[index]), ALARM_LABELS[index]))

        # Paint each band only between its own bounds. Shading every band from
        # the ego outwards would stack three tints into a muddy block.
        levels.sort(reverse=True)
        for index, (distance, colour, label) in enumerate(levels):
            outer = min(distance, self.max_range)
            inner = min(levels[index + 1][0], self.max_range) if index + 1 < len(levels) else 0.0
            y = self.to_pixel_y(outer)
            y_inner = self.to_pixel_y(inner)

            band = QColor(colour)
            band.setAlpha(22)
            painter.fillRect(QRectF(rect.left(), y, rect.width(), y_inner - y), band)

            pen_colour = QColor(colour)
            pen_colour.setAlpha(150)
            pen = QPen(pen_colour, 1, Qt.DashLine)
            painter.setPen(pen)
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))

            painter.setFont(QFont("Helvetica Neue", 8, QFont.DemiBold))
            painter.setPen(QPen(colour))
            painter.drawText(QRectF(rect.left() + 6, y - 14, 90, 13),
                             Qt.AlignLeft | Qt.AlignVCenter, f"{label}  {distance:g} m")

    def _sector_path(self, r_lo, r_hi, az_lo, az_hi):
        """Annular sector between two ranges and two azimuths (deg, +right).

        Both axes share one metres-per-pixel scale, so circles stay circles.
        Qt measures angles anticlockwise from 3 o'clock; boresight is 12.
        """
        origin = QPointF(self.to_pixel_x(0.0), self.to_pixel_y(0.0))
        scale = self.scale()

        def ring(radius_m):
            r = radius_m * scale
            return QRectF(origin.x() - r, origin.y() - r, 2 * r, 2 * r)

        def at(radius_m, az):
            a = math.radians(az)
            return QPointF(self.to_pixel_x(radius_m * math.sin(a)),
                           self.to_pixel_y(radius_m * math.cos(a)))

        # arcTo joins the current point to the arc's start with a straight
        # line, which draws the radial edges.
        path = QPainterPath(at(r_lo, az_lo))
        path.arcTo(ring(r_hi), 90.0 - az_lo, -(az_hi - az_lo))
        if r_lo > 0:
            path.arcTo(ring(r_lo), 90.0 - az_hi, az_hi - az_lo)
        path.closeSubpath()
        return path

    def _draw_coverage(self, painter):
        """Every cell the radar has detected something in, shaded by how often.

        This is the area the radar actually covers on this mounting - built up
        from its own detections, not taken from a datasheet.
        """
        if self.scene is None or not self.scene.coverage.total:
            return
        coverage = self.scene.coverage
        log_peak = math.log1p(coverage.peak)
        painter.setPen(Qt.NoPen)
        for r_lo, r_hi, az_lo, az_hi, count in coverage.cells():
            if r_lo > self.max_range:
                continue
            colour = QColor(self.c('coverage'))
            colour.setAlpha(int(70 + 150 * math.log1p(count) / log_peak))
            painter.setBrush(QBrush(colour))
            painter.drawPath(self._sector_path(r_lo, r_hi, az_lo, az_hi))

    def _draw_fov(self, painter):
        """Nominal field of view (assumed) and the observed extent (measured)."""
        painter.setBrush(Qt.NoBrush)
        fov = NOMINAL_FOV_DEG
        reach = min(NOMINAL_RANGE_M, self.max_range * 1.5)
        painter.setPen(QPen(self.c('fov', 150), 1.2, Qt.DashLine))
        painter.drawPath(self._sector_path(0.0, reach, -fov, fov))

        extent = self.scene.coverage.observed_extent() if self.scene else None
        if extent:
            az_lo, az_hi, r_max = extent
            painter.setPen(QPen(self.c('coverage', 220), 1.6))
            painter.drawPath(self._sector_path(0.0, r_max, az_lo, az_hi))
            tip = math.radians((az_lo + az_hi) / 2.0)
            label_r = min(r_max, self.max_range * 0.92)
            self._draw_label(
                painter, f"seen {az_lo:+.0f}° to {az_hi:+.0f}°, {r_max:.0f} m",
                self.to_pixel_x(label_r * math.sin(tip)) + 8,
                self.to_pixel_y(label_r * math.cos(tip)), self.c('coverage'))

        rect = self.plot_rect()
        self._draw_label(painter, f"nominal ±{fov:.0f}° (assumed)",
                         rect.left() + 8, rect.top() + 28, self.c('fov'))

    def _draw_boresight(self, painter):
        rect = self.plot_rect()
        x = self.to_pixel_x(0.0)
        pen = QPen(self.c('track', 55), 1, Qt.DashLine)
        painter.setPen(pen)
        painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))

    def _doppler_colour(self, doppler):
        """Warm for closing targets, cool for opening ones."""
        if doppler is None:
            return self.c('point_still')
        if doppler < -1.0:
            return self.c('point_near')
        if doppler > 1.0:
            return self.c('point_away')
        return self.c('point_still')

    def _draw_points(self, painter):
        painter.setPen(Qt.NoPen)
        if self.scene is not None:
            # Coloured by motion over the ground, with ego speed removed
            for point in self.scene.points:
                if not self._in_view(point['lateral'], point['range']):
                    continue
                colour = QColor(self.c(class_role(point)))
                colour.setAlpha(235)
                painter.setBrush(QBrush(colour))
                painter.drawEllipse(QPointF(self.to_pixel_x(point['lateral']),
                                            self.to_pixel_y(point['range'])), 3.4, 3.4)
            return
        for point in self.detected_points:
            try:
                lateral = float(point.get('D_X', 0.0))
                range_m = float(point.get('D_Y', 0.0))
            except (TypeError, ValueError):
                continue
            if not self._in_view(lateral, range_m):
                continue

            colour = QColor(self._doppler_colour(point.get('Doppler')))
            colour.setAlpha(225)
            painter.setBrush(QBrush(colour))
            centre = QPointF(self.to_pixel_x(lateral), self.to_pixel_y(range_m))
            painter.drawEllipse(centre, 3.0, 3.0)

    def _draw_clusters(self, painter):
        for cluster in self.clusters:
            try:
                lateral = float(cluster.get('C_X', 0.0))
                range_m = float(cluster.get('C_Y', 0.0))
            except (TypeError, ValueError):
                continue
            if not self._in_view(lateral, range_m):
                continue

            half_x = float(cluster.get('C_SX', 0.0) or 0.0)
            half_y = float(cluster.get('C_SY', 0.0) or 0.0)
            width = max(self.metres_to_pixels_x(half_x) * 2.0, 14.0)
            height = max(self.metres_to_pixels_y(half_y) * 2.0, 14.0)
            centre = QPointF(self.to_pixel_x(lateral), self.to_pixel_y(range_m))

            fill = QColor(self.c('cluster'))
            fill.setAlpha(34)
            edge = QColor(self.c('cluster'))
            edge.setAlpha(170)
            painter.setBrush(QBrush(fill))
            painter.setPen(QPen(edge, 1.2))
            painter.drawRoundedRect(
                QRectF(centre.x() - width / 2, centre.y() - height / 2, width, height),
                5, 5)

    def _draw_tracks(self, painter):
        font = QFont("Helvetica Neue", 8, QFont.DemiBold)
        painter.setFont(font)

        for index, track in enumerate(self.tracks):
            try:
                lateral = float(track.get('T_X', 0.0))
                range_m = float(track.get('T_Y', 0.0))
                v_lateral = float(track.get('T_Vx', 0.0) or 0.0)
                v_range = float(track.get('T_Vy', 0.0) or 0.0)
            except (TypeError, ValueError):
                continue
            if not self._in_view(lateral, range_m):
                continue

            centre = QPointF(self.to_pixel_x(lateral), self.to_pixel_y(range_m))

            ego = self.scene.ego_mps if self.scene is not None else 0.0
            ground, moving = classify_track(v_lateral, v_range, ego)
            colour = self.c('track' if moving else 'track_static')

            halo = QColor(colour)
            halo.setAlpha(38)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(halo))
            painter.drawEllipse(centre, 15, 15)

            heading, _ = self.object_heading(v_range, v_lateral)
            length, width = self.object_footprint(lateral, range_m)
            self._draw_oriented_box(painter, centre, length, width, heading,
                                    colour, fill_alpha=30, line_width=1.5)
            painter.setBrush(QBrush(colour))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(centre, 2.4, 2.4)

            self._draw_velocity_arrow(painter, centre, v_lateral, v_range, colour)

            # Identity assigned by association, not reported by the radar
            track_id = track.get('T_ID', index + 1)
            state = "MOVING" if moving else "STATIC"
            label = f"ID {track_id} {state}  {range_m:.1f} m  {ground:.1f} m/s"
            self._draw_label(painter, label, centre.x() + 13, centre.y() - 20, colour)

    # Seconds of travel the velocity arrow represents, and its pixel cap so a
    # fast closing target does not draw a line across the whole scene.
    VELOCITY_HORIZON_S = 0.6
    MAX_ARROW_PX = 90.0

    def _draw_velocity_arrow(self, painter, origin, v_lateral, v_range, colour):
        """Arrow showing where the target travels over the next VELOCITY_HORIZON_S."""
        if abs(v_lateral) < 0.05 and abs(v_range) < 0.05:
            return
        dx = self.metres_to_pixels_x(v_lateral * self.VELOCITY_HORIZON_S)
        dy = -self.metres_to_pixels_y(v_range * self.VELOCITY_HORIZON_S)
        span = (dx * dx + dy * dy) ** 0.5
        if span > self.MAX_ARROW_PX:
            scale = self.MAX_ARROW_PX / span
            dx, dy = dx * scale, dy * scale
        tip = QPointF(origin.x() + dx, origin.y() + dy)

        painter.setPen(QPen(colour, 1.6))
        painter.setBrush(Qt.NoBrush)
        painter.drawLine(origin, tip)

        dx, dy = tip.x() - origin.x(), tip.y() - origin.y()
        length = max((dx * dx + dy * dy) ** 0.5, 1e-6)
        if length < 6:
            return
        ux, uy = dx / length, dy / length
        head = 7.0
        left = QPointF(tip.x() - ux * head - uy * head * 0.45,
                       tip.y() - uy * head + ux * head * 0.45)
        right = QPointF(tip.x() - ux * head + uy * head * 0.45,
                        tip.y() - uy * head - ux * head * 0.45)
        painter.setBrush(QBrush(colour))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(QPolygonF([tip, left, right]))

    # Shortest range the bench radar has been observed to report. Below this
    # it returns a byte-identical value, so it is a clamp rather than a
    # measurement - a reading at the floor means "this close or closer", not a
    # distance. Confirm against the sensor's minimum-range specification.
    SENSOR_MIN_RANGE_M = 0.6

    # Default footprint when the radar reports no size for an object.
    DEFAULT_LENGTH_M = 4.5
    DEFAULT_WIDTH_M = 1.8
    STATIONARY_SPEED_MPS = 0.6

    def object_heading(self, v_long, v_lat):
        """Heading in radians, 0 = directly away from ego, measured toward +lateral.

        The radar reports velocity relative to the ego vehicle, so a car ahead
        travelling the same way shows a small negative longitudinal velocity.
        Adding ego speed back recovers the target's motion over the ground,
        which is a far better stand-in for orientation. Returns the heading and
        whether it came from absolute or relative motion - the caller labels it,
        because neither is a measured orientation.
        """
        if self.ego_speed_mps is not None:
            absolute_long = v_long + self.ego_speed_mps
            basis = "abs"
        else:
            absolute_long, basis = v_long, "rel"

        # Below the stationary threshold there is no direction of travel to
        # infer from, and a parked object still has an orientation we cannot
        # see. Report nothing rather than a heading built from noise.
        if math.hypot(absolute_long, v_lat) < self.STATIONARY_SPEED_MPS:
            return None, basis
        return math.atan2(v_lat, absolute_long), basis

    def dynamic_property(self, v_long, v_lat):
        """moving / stationary, derived from motion over the ground."""
        if self.ego_speed_mps is None:
            return "unknown"
        speed = math.hypot(v_long + self.ego_speed_mps, v_lat)
        return "stationary" if speed < self.STATIONARY_SPEED_MPS else "moving"

    def _draw_oriented_box(self, painter, centre, length_m, width_m, heading,
                           colour, fill_alpha=42, line_width=1.8):
        """Footprint rotated to the object's heading.

        Scene axes are anisotropic - lateral and range use different metres per
        pixel - so the corners are rotated in metres and only then converted to
        pixels. Rotating the pixel rectangle instead would shear the box.
        """
        half_l, half_w = length_m / 2.0, width_m / 2.0
        corners_m = [(+half_l, -half_w), (+half_l, +half_w),
                     (-half_l, +half_w), (-half_l, -half_w)]

        angle = 0.0 if heading is None else heading
        cos_a, sin_a = math.cos(angle), math.sin(angle)

        polygon = QPolygonF()
        for along, across in corners_m:
            d_long = along * cos_a - across * sin_a
            d_lat = along * sin_a + across * cos_a
            polygon.append(QPointF(centre.x() + self.metres_to_pixels_x(d_lat),
                                   centre.y() - self.metres_to_pixels_y(d_long)))

        fill = QColor(colour)
        fill.setAlpha(fill_alpha)
        painter.setBrush(QBrush(fill))
        painter.setPen(QPen(colour, line_width))
        painter.drawPolygon(polygon)

        # Short spur from the centre to the front face, marking which way it faces
        if heading is not None:
            nose_long, nose_lat = half_l * cos_a, half_l * sin_a
            painter.setPen(QPen(colour, line_width))
            painter.drawLine(centre, QPointF(
                centre.x() + self.metres_to_pixels_x(nose_lat),
                centre.y() - self.metres_to_pixels_y(nose_long)))

    def _draw_objects_of_interest(self, painter):
        if self.static_obj:
            self._draw_ooi(painter, self.static_obj, self.c('static_ooi'), "STATIC", square=True)
        if self.dynamic_obj:
            self._draw_ooi(painter, self.dynamic_obj, self.c('dynamic_ooi'), "MOVING", square=False)

    def _draw_ooi(self, painter, obj, colour, tag, square):
        lateral, range_m = obj['lateral'], obj['range']
        if not self._in_view(lateral, range_m):
            return
        centre = QPointF(self.to_pixel_x(lateral), self.to_pixel_y(range_m))

        glow = QColor(colour)
        glow.setAlpha(38)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(glow))
        painter.drawEllipse(centre, 21, 21)

        heading, _ = self.object_heading(obj['vx'], obj['vy'])
        length, width = self.object_footprint(lateral, range_m)
        self._draw_oriented_box(painter, centre, length, width, heading,
                                colour, fill_alpha=60, line_width=2.0)

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(colour))
        painter.drawEllipse(centre, 2.6, 2.6)

        self._draw_velocity_arrow(painter, centre, obj['vy'], obj['vx'], colour)

        label = f"{tag}  {range_m:.1f} / {lateral:+.1f} m"
        self._draw_label(painter, label, centre.x() + 14, centre.y() + 10, colour)

    def object_footprint(self, lateral, range_m, max_gap_m=3.0):
        """Length and width from the nearest cluster's extent, else a default.

        The radar sends no object dimensions, so a cluster sitting on top of the
        object is the only size evidence available.
        """
        best, best_gap = None, max_gap_m
        for cluster in self.clusters:
            try:
                gap = math.hypot(float(cluster.get('C_X', 0.0)) - lateral,
                                 float(cluster.get('C_Y', 0.0)) - range_m)
            except (TypeError, ValueError):
                continue
            if gap < best_gap:
                best, best_gap = cluster, gap

        if best is None:
            return self.DEFAULT_LENGTH_M, self.DEFAULT_WIDTH_M
        length = float(best.get('C_SY', 0.0) or 0.0) or self.DEFAULT_LENGTH_M
        width = float(best.get('C_SX', 0.0) or 0.0) or self.DEFAULT_WIDTH_M
        return max(length, 1.0), max(width, 0.6)

    def _draw_label(self, painter, text, x, y, colour):
        painter.setFont(QFont("Helvetica Neue", 8, QFont.DemiBold))
        metrics = QFontMetrics(painter.font())
        width = metrics.horizontalAdvance(text) + 10
        height = metrics.height() + 4

        rect = self.plot_rect()
        x = min(x, rect.right() - width - 2)
        y = max(y, rect.top() + 2)

        box = QRectF(x, y, width, height)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(self.c('label_bg', LABEL_BG_ALPHA[self.theme])))
        painter.drawRoundedRect(box, 3, 3)
        painter.setPen(QPen(colour))
        painter.drawText(box, Qt.AlignCenter, text)

    def _draw_ego(self, painter):
        """Ego vehicle marker sitting at the origin of the scene."""
        rect = self.plot_rect()
        x = self.to_pixel_x(0.0)
        y = rect.bottom()

        body = QPolygonF([
            QPointF(x, y - 13),
            QPointF(x - 8, y + 3),
            QPointF(x, y - 1),
            QPointF(x + 8, y + 3),
        ])
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(self.c('ego'), 1.3))
        fill = QColor(self.c('ego'))
        fill.setAlpha(70)
        painter.setBrush(QBrush(fill))
        painter.drawPolygon(body)

    def _draw_legend(self, painter):
        entries = [
            (self.c('point_near'), "closing"),
            (self.c('point_still'), "static"),
            (self.c('point_away'), "opening"),
            (self.c('track'), "moving track"),
            (self.c('track_static'), "static track"),
            (self.c('coverage'), "area covered"),
            (self.c('dynamic_ooi'), "object of interest"),
        ]
        painter.setFont(QFont("Helvetica Neue", 8))
        metrics = QFontMetrics(painter.font())

        rect = self.plot_rect()
        x = rect.left() + 10
        y = rect.top() + 10
        swatch = 7

        total = sum(metrics.horizontalAdvance(text) + swatch + 16 for _, text in entries)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(self.c('label_bg', LABEL_BG_ALPHA[self.theme] - 15)))
        painter.drawRoundedRect(QRectF(x - 6, y - 5, total + 8, 20), 4, 4)

        for colour, text in entries:
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(colour))
            painter.drawEllipse(QPointF(x + swatch / 2, y + 5), swatch / 2, swatch / 2)
            painter.setPen(QPen(self.c('text_mid')))
            painter.drawText(QRectF(x + swatch + 5, y - 2, metrics.horizontalAdvance(text) + 6, 15),
                             Qt.AlignLeft | Qt.AlignVCenter, text)
            x += metrics.horizontalAdvance(text) + swatch + 16

    def _in_view(self, lateral, range_m):
        return (abs(lateral) <= self.half_width + 2.0
                and -1.0 <= range_m <= self.max_range + 2.0)

    # ── Interaction ────────────────────────────────────────────────────────

    def wheelEvent(self, event):
        """Scroll to zoom the range axis."""
        steps = event.angleDelta().y() / 120.0
        if not steps:
            return
        factor = 0.9 ** steps
        self.max_range = max(20.0, min(300.0, self.max_range * factor))
        self.grid_step = 5.0 if self.max_range <= 60 else (
            10.0 if self.max_range <= 160 else 25.0)
        self._grid_cache = None
        self.update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._grid_cache = None


class ScatterPanel(QWidget):
    """Range against Doppler or azimuth, drawn from the recent point history.

    The radar sends a sparse point list over CAN, not the range-Doppler or
    range-azimuth heatmap, so these panels plot detections rather than
    intensity. Older frames fade so a moving target leaves a short trail.
    """

    MARGIN_LEFT = 40
    MARGIN_RIGHT = 12
    MARGIN_TOP = 28
    MARGIN_BOTTOM = 32

    def __init__(self, title, x_label, x_key, x_limit=None,
                 theme=DEFAULT_THEME, parent=None):
        super(ScatterPanel, self).__init__(parent)
        self.setMinimumSize(260, 220)
        self.title = title
        self.x_label = x_label
        self.x_key = x_key          # 'doppler' or 'azimuth'
        self.fixed_x_limit = x_limit
        self.theme = theme
        self.scene = None
        # Axis limits only grow, so the scale does not jump frame to frame
        self.x_limit = x_limit or 3.0
        self.range_limit = 20.0

    def c(self, role, alpha=None):
        return theme_colour(self.theme, role, alpha)

    def set_theme(self, theme):
        self.theme = theme
        self.update()

    def set_scene(self, scene):
        self.scene = scene
        for frame in scene.history:
            for point in frame:
                self.range_limit = max(self.range_limit,
                                       nice_ceiling(point['radial'] * 1.05))
                if self.fixed_x_limit is None:
                    self.x_limit = max(self.x_limit,
                                       nice_ceiling(abs(point[self.x_key]) * 1.2))
        self.update()

    def reset_scale(self):
        self.x_limit = self.fixed_x_limit or 3.0
        self.range_limit = 20.0
        self.update()

    def plot_rect(self):
        return QRectF(self.MARGIN_LEFT, self.MARGIN_TOP,
                      max(1, self.width() - self.MARGIN_LEFT - self.MARGIN_RIGHT),
                      max(1, self.height() - self.MARGIN_TOP - self.MARGIN_BOTTOM))

    def to_pixel(self, x_value, range_m):
        rect = self.plot_rect()
        px = rect.center().x() + (x_value / self.x_limit) * rect.width() / 2.0
        py = rect.bottom() - (range_m / self.range_limit) * rect.height()
        return QPointF(px, py)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), self.c('bg_deep'))
        rect = self.plot_rect()
        painter.fillRect(rect, self.c('bg_plot'))
        painter.setFont(QFont("Helvetica Neue", 8))

        # Grid: five range lines, and x ticks at quarters of the limit
        for i in range(0, 5):
            range_m = self.range_limit * i / 4.0
            y = self.to_pixel(0, range_m).y()
            painter.setPen(QPen(self.c('grid_minor'), 1))
            painter.drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y))
            painter.setPen(QPen(self.c('text_dim')))
            painter.drawText(QRectF(0, y - 8, self.MARGIN_LEFT - 5, 16),
                             Qt.AlignRight | Qt.AlignVCenter, f"{range_m:g}")
        for i in range(-2, 3):
            x_value = self.x_limit * i / 2.0
            x = self.to_pixel(x_value, 0).x()
            painter.setPen(QPen(self.c('axis_line') if i == 0 else self.c('grid_minor'), 1))
            painter.drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()))
            painter.setPen(QPen(self.c('text_dim')))
            painter.drawText(QRectF(x - 25, rect.bottom() + 3, 50, 14),
                             Qt.AlignCenter, f"{x_value:g}")

        painter.setPen(QPen(self.c('panel_border'), 1))
        painter.drawRect(rect)
        painter.setPen(QPen(self.c('text_mid')))
        painter.setFont(QFont("Helvetica Neue", 9, QFont.DemiBold))
        painter.drawText(QRectF(rect.left(), 4, rect.width(), 18),
                         Qt.AlignCenter, self.title.upper())
        painter.setFont(QFont("Helvetica Neue", 8))
        painter.setPen(QPen(self.c('text_dim')))
        painter.drawText(QRectF(rect.left(), rect.bottom() + 16, rect.width(), 14),
                         Qt.AlignCenter, self.x_label)
        painter.drawText(QRectF(2, 4, self.MARGIN_LEFT, 18),
                         Qt.AlignLeft | Qt.AlignVCenter, "m")

        if self.scene is not None:
            painter.setRenderHint(QPainter.Antialiasing, True)
            painter.setClipRect(rect)
            self._draw_history(painter)
            self._draw_tracks(painter)
        painter.end()

    def _draw_history(self, painter):
        frames = list(self.scene.history)
        painter.setPen(Qt.NoPen)
        for age, frame in enumerate(reversed(frames)):
            alpha = int(235 * (1.0 - age / max(len(frames), 1)) ** 1.5) + 20
            radius = 3.4 if age == 0 else 2.4
            for point in frame:
                colour = QColor(self.c(class_role(point)))
                colour.setAlpha(alpha)
                painter.setBrush(QBrush(colour))
                painter.drawEllipse(self.to_pixel(point[self.x_key], point['radial']),
                                    radius, radius)

    def _draw_tracks(self, painter):
        painter.setBrush(Qt.NoBrush)
        for track in self.scene.tracks:
            if self.x_key == 'doppler':
                # Radial component of the track's relative velocity
                radial = max(track['radial'], 1e-6)
                x_value = (track['v_lateral'] * track['lateral']
                           + track['v_range'] * track['range']) / radial
            else:
                x_value = track['azimuth']
            role = 'track' if track['moving'] else 'track_static'
            painter.setPen(QPen(self.c(role), 1.6))
            painter.drawEllipse(self.to_pixel(x_value, track['radial']), 7, 7)


class DataTableWidget(QFrame):
    """Compact readout card: a title above one row of values."""

    def __init__(self, title, headers, parent=None):
        super(DataTableWidget, self).__init__(parent)
        self.setObjectName("card")
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        title_label = QLabel(title.upper())
        title_label.setObjectName("cardTitle")
        title_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(title_label)
        self._title = title_label
        self._base_title = title.upper()
        self._accent = QBrush(QColor('#E6EDF7'))
        self._stale_colour = QBrush(QColor('#6A7A92'))

        self.table = QTableWidget(1, len(headers))
        self.table.setHorizontalHeaderLabels(headers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setFixedHeight(52)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setFocusPolicy(Qt.NoFocus)
        self.table.setSelectionMode(QTableWidget.NoSelection)
        self.table.setShowGrid(False)
        layout.addWidget(self.table)
        self.setLayout(layout)

        self.table_items = []
        for column in range(len(headers)):
            item = QTableWidgetItem("—")
            item.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(0, column, item)
            self.table_items.append(item)

    def update_data(self, data_list):
        for column, value in enumerate(data_list):
            if column < len(self.table_items):
                self.table_items[column].setText(str(value))

    def set_stale(self, stale, age=None):
        """Mark the card as showing a value that stopped arriving.

        Object messages are sent only while the radar has something to report,
        so a panel can keep displaying a reading long after the object has
        gone. Holding the last value silently makes a stale number
        indistinguishable from a live one.
        """
        self._title.setText(self._base_title if not stale
                            else f"{self._base_title}   ·   STALE {age:.0f}s")
        self._title.setProperty("stale", "yes" if stale else "no")
        self._title.style().unpolish(self._title)
        self._title.style().polish(self._title)
        for item in self.table_items:
            item.setForeground(QBrush(self._stale_colour if stale
                                      else self._accent))

    def set_accent(self, colour):
        self._accent = QBrush(colour)
        for item in self.table_items:
            item.setForeground(self._accent)


class AttributeCard(QFrame):
    """Key/value readout for one object, in the order of the object spec.

    Fields the radar does not transmit are shown as em-dashes rather than
    omitted: a missing attribute is information, and hiding the row would make
    the gap invisible.
    """

    def __init__(self, title, fields, parent=None):
        super(AttributeCard, self).__init__(parent)
        self.setObjectName("card")

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 8)
        layout.setSpacing(0)

        title_label = QLabel(title.upper())
        title_label.setObjectName("cardTitle")
        layout.addWidget(title_label)
        self._title = title_label
        self._base_title = title.upper()

        self.value_labels = {}
        for key, caption in fields:
            row = QWidget()
            row_layout = QHBoxLayout()
            row_layout.setContentsMargins(12, 1, 12, 1)
            row_layout.setSpacing(6)
            row.setLayout(row_layout)

            name = QLabel(caption)
            name.setObjectName("attrName")
            value = QLabel("—")
            value.setObjectName("attrValue")
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            row_layout.addWidget(name)
            row_layout.addStretch()
            row_layout.addWidget(value)
            layout.addWidget(row)

            self.value_labels[key] = value

        self.setLayout(layout)

    def set_stale(self, stale, age=None):
        self._title.setText(self._base_title if not stale
                            else f"{self._base_title}   ·   STALE {age:.0f}s")
        self._title.setProperty("stale", "yes" if stale else "no")
        self._title.style().unpolish(self._title)
        self._title.style().polish(self._title)
        for label in self.value_labels.values():
            label.setProperty("stale", "yes" if stale else "no")
            label.style().unpolish(label)
            label.style().polish(label)

    def update_values(self, values):
        for key, label in self.value_labels.items():
            value = values.get(key)
            label.setText("—" if value in (None, "") else str(value))
            label.setProperty("missing", "yes" if value in (None, "") else "no")
            label.style().unpolish(label)
            label.style().polish(label)


class RadarVisualizerMainWindow(QMainWindow):
    """Main window: radar scene on the left, readouts on the right."""

    # Frames without an alarm message before the banner clears
    ALARM_CLEAR_FRAMES = 4

    def __init__(self, theme=None):
        super(RadarVisualizerMainWindow, self).__init__()
        self.setWindowTitle("Orion Radar Visualizer")
        self.setGeometry(60, 60, 1760, 940)

        self.settings = QSettings("Starkenn", "OrionRadarVisualizer")
        stored = self.settings.value("theme", DEFAULT_THEME)
        self.theme = theme or (stored if stored in THEMES else DEFAULT_THEME)

        self._frame_times = deque(maxlen=40)
        self._frames_seen = 0
        self._frames_without_alarm = 0
        self._connected = False
        self._radar_cycle = None
        self._cycle_ever_nonzero = False
        # When each section last carried data, so a panel can say when the
        # value on it stopped being refreshed.
        self._last_seen = {}
        # Classification, point history and coverage, shared by every view
        self.scene = SceneModel()

        self.setup_ui()

        self.socket_thread = SocketClientThread()
        self.socket_thread.data_received.connect(self.update_with_json)
        self.socket_thread.connection_changed.connect(self.set_connected)
        self.socket_thread.start()

        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.refresh_status)
        self.status_timer.start(200)

    def c(self, role, alpha=None):
        """Colour for a role in the active theme."""
        return theme_colour(self.theme, role, alpha)

    def toggle_theme(self):
        self.apply_theme('light' if self.theme == 'dark' else 'dark')

    def apply_theme(self, theme):
        """Switch palette, restyle every widget and remember the choice."""
        if theme not in THEMES:
            return
        self.theme = theme
        self.settings.setValue("theme", theme)

        self.setStyleSheet(build_style_sheet(theme))
        self.plot_widget.set_theme(theme)
        for panel in (self.doppler_panel, self.azimuth_panel):
            panel.set_theme(theme)
        self.theme_button.setText("LIGHT" if theme == 'dark' else "DARK")
        self._apply_table_accents()

        for widget in (self.connection_label, self.alarm_banner, self.theme_button,
                       self.reset_button):
            self._restyle(widget)

    def reset_coverage(self):
        """Forget the area covered so far, e.g. after moving the radar."""
        self.scene.reset_coverage()
        for panel in (self.doppler_panel, self.azimuth_panel):
            panel.reset_scale()
        self.plot_widget.update()

    def _apply_table_accents(self):
        self.static_alarm_table.set_accent(self.c('text_mid'))
        self.dynamic_alarm_table.set_accent(self.c('text_mid'))
        self.static_obj_table.set_accent(self.c('static_ooi'))
        self.dynamic_obj_table.set_accent(self.c('dynamic_ooi'))
        self.coverage_table.set_accent(self.c('coverage'))
        for card in (self.class_table, self.stream_table, self.counts_reported):
            card.set_accent(self.c('text_bright'))

    # ── Layout ─────────────────────────────────────────────────────────────

    def setup_ui(self):
        self.setStyleSheet(build_style_sheet(self.theme))

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout()
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        central.setLayout(root)

        root.addWidget(self._build_header())

        body = QWidget()
        body_layout = QHBoxLayout()
        body_layout.setContentsMargins(12, 10, 12, 12)
        body_layout.setSpacing(12)
        body.setLayout(body_layout)
        root.addWidget(body, 1)

        # Range-Doppler and range-azimuth stacked to the left of the scene
        left = QWidget()
        left.setFixedWidth(340)
        left_layout = QVBoxLayout()
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)
        left.setLayout(left_layout)
        self.doppler_panel = ScatterPanel(
            "Range - Doppler", "radial velocity (m/s, closing < 0)", 'doppler',
            theme=self.theme)
        self.azimuth_panel = ScatterPanel(
            "Range - Azimuth", "azimuth (deg, right > 0)", 'azimuth',
            x_limit=90.0, theme=self.theme)
        left_layout.addWidget(self.doppler_panel, 1)
        left_layout.addWidget(self.azimuth_panel, 1)
        body_layout.addWidget(left)

        self.plot_widget = RadarPlotWidget(theme=self.theme)
        self.plot_widget.scene = self.scene
        body_layout.addWidget(self.plot_widget, 1)
        body_layout.addWidget(self._build_side_panel())

    def _build_header(self):
        header = QWidget()
        header.setObjectName("header")
        header.setFixedHeight(62)
        layout = QHBoxLayout()
        layout.setContentsMargins(16, 0, 16, 0)
        layout.setSpacing(18)
        header.setLayout(layout)

        title = QLabel("ORION RADAR")
        title.setObjectName("appTitle")
        layout.addWidget(title)

        self.connection_label = QLabel("DISCONNECTED")
        self.connection_label.setObjectName("statusChip")
        self.connection_label.setProperty("state", "off")
        self.connection_label.setAlignment(Qt.AlignCenter)
        self.connection_label.setFixedHeight(26)
        layout.addWidget(self.connection_label, 0, Qt.AlignVCenter)

        layout.addStretch()

        self.reset_button = QPushButton("RESET MAP")
        self.reset_button.setObjectName("themeToggle")
        self.reset_button.setCursor(Qt.PointingHandCursor)
        self.reset_button.setFixedSize(104, 26)
        self.reset_button.setToolTip("Clear the area-covered map and panel scales")
        self.reset_button.clicked.connect(self.reset_coverage)
        layout.addWidget(self.reset_button)

        self.theme_button = QPushButton("LIGHT" if self.theme == 'dark' else "DARK")
        self.theme_button.setObjectName("themeToggle")
        self.theme_button.setCursor(Qt.PointingHandCursor)
        self.theme_button.setFixedSize(76, 26)
        self.theme_button.setToolTip("Switch between light and dark appearance")
        self.theme_button.clicked.connect(self.toggle_theme)
        layout.addWidget(self.theme_button)

        self.alarm_banner = QLabel("NO ALARM")
        self.alarm_banner.setObjectName("alarmBanner")
        self.alarm_banner.setProperty("level", "0")
        self.alarm_banner.setAlignment(Qt.AlignCenter)
        self.alarm_banner.setFixedSize(190, 34)
        layout.addWidget(self.alarm_banner, 0, Qt.AlignVCenter)

        speed_box = QWidget()
        speed_layout = QVBoxLayout()
        speed_layout.setContentsMargins(0, 0, 0, 0)
        speed_layout.setSpacing(0)
        speed_box.setLayout(speed_layout)

        self.ego_speed_label = QLabel("—")
        self.ego_speed_label.setObjectName("egoSpeed")
        self.ego_speed_label.setAlignment(Qt.AlignRight | Qt.AlignBottom)
        self.speed_caption = QLabel("EGO SPEED km/h")
        self.speed_caption.setObjectName("caption")
        self.speed_caption.setAlignment(Qt.AlignRight | Qt.AlignTop)
        speed_layout.addWidget(self.ego_speed_label)
        speed_layout.addWidget(self.speed_caption)
        layout.addWidget(speed_box)

        return header

    def _build_side_panel(self):
        panel = QWidget()
        panel.setObjectName("sidePanel")
        panel.setFixedWidth(320)
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        panel.setLayout(layout)

        self.class_table = DataTableWidget(
            "Moving / static", ["MOV PTS", "STA PTS", "MOV TRK", "STA TRK"])
        self.coverage_table = DataTableWidget(
            "Area covered", ["AZ FROM°", "AZ TO°", "RANGE m"])
        self.static_obj_table = DataTableWidget(
            "Static object of interest", ["RANGE m", "LATERAL m"])
        self.dynamic_obj_table = DataTableWidget(
            "Moving object of interest", ["RANGE m", "LATERAL m", "Vx m/s", "Vy m/s"])
        self.static_alarm_table = DataTableWidget(
            "Static alarm thresholds", ["L1 m", "L2 m", "L3 m"])
        self.dynamic_alarm_table = DataTableWidget(
            "Moving alarm thresholds", ["L1 m", "L2 m", "L3 m"])
        self.counts_reported = DataTableWidget(
            "Objects reported by radar", ["DYNAMIC", "STATIC"])
        self.object_card = AttributeCard("Moving object of interest", [
            ("objId", "objId"),
            ("distX", "distX  (m)"),
            ("distY", "distY  (m)"),
            ("vrelX", "vrelX  (m/s)"),
            ("vrelY", "vrelY  (m/s)"),
            ("arelX", "arelX  (m/s²)"),
            ("arelY", "arelY  (m/s²)"),
            ("orientation", "orientation"),
            ("dynamic_property", "dynamic_property"),
            ("classification", "classification"),
            ("size", "length × width (m)"),
        ])
        # Unscaled LSB counts straight off the bus, so a suspect reading can
        # be checked against the DBC scaling without opening the log.
        self.raw_card = AttributeCard("Raw signal - moving object 1", [
            ("x_raw", "distX raw (LSB)"),
            ("x_check", "0.032 x raw - 60.8"),
            ("y_raw", "distY raw (LSB)"),
            ("y_check", "0.064 x raw - 131.072"),
            ("vx_raw", "vrelX raw (LSB)"),
            ("vy_raw", "vrelY raw (LSB)"),
            ("ttc", "time to collision (s)"),
            ("yaw", "yaw rate"),
        ])
        self.bus_card = AttributeCard("Bus health", [
            ("objects", "objects 1"),
            ("objects 2/3", "objects 2/3"),
            ("counts", "object counts"),
            ("alarm", "alarm + thresholds"),
            ("ttc", "time to collision"),
            ("ego speed", "ego speed"),
            ("ego yaw", "ego yaw"),
            ("HIL truth", "HIL ground truth"),
            ("radar stream", "radar TLV stream"),
        ])
        self.stream_table = DataTableWidget(
            "Stream", ["FRAMES", "FPS", "RADAR CYCLE"])

        for card in (self.class_table, self.coverage_table, self.counts_reported,
                     self.object_card, self.raw_card, self.dynamic_alarm_table,
                     self.bus_card, self.stream_table):
            layout.addWidget(card)

        self._apply_table_accents()

        layout.addStretch()

        self.timestamp_label = QLabel("—")
        self.timestamp_label.setObjectName("timestamp")
        self.timestamp_label.setAlignment(Qt.AlignLeft)
        layout.addWidget(self.timestamp_label)

        return panel

    # ── Updates ────────────────────────────────────────────────────────────

    def format_value(self, value, decimal_places=1):
        try:
            if value in ('N/A', None, ''):
                return '—'
            return f"{float(value):.{decimal_places}f}"
        except (ValueError, TypeError):
            return str(value)

    def set_connected(self, connected):
        if connected == self._connected:
            return
        self._connected = connected
        self.connection_label.setText("LIVE" if connected else "DISCONNECTED")
        self.connection_label.setProperty("state", "on" if connected else "off")
        self._restyle(self.connection_label)

    def update_with_json(self, json_data):
        """Render one frame. No single missing message may abort the update."""
        try:
            self._frames_seen += 1
            self._frame_times.append(time.monotonic())

            self.scene.update(json_data)
            self.plot_widget.update_data(json_data)
            for panel in (self.doppler_panel, self.azimuth_panel):
                panel.set_scene(self.scene)
            self._update_ego_speed(json_data)
            self._update_alarm(json_data)
            self._update_tables(json_data)
        except Exception as e:
            print(f"Error updating display: {e}")

    def _update_ego_speed(self, json_data):
        try:
            # The signal is m/s (DBC Vehicle_Data_Speed); shown as km/h
            speed_mps = float((json_data.get('ego_speed') or {}).get('ego_speed'))
        except (TypeError, ValueError, AttributeError):
            return  # keep the last known speed rather than blanking it
        if speed_mps >= 0:
            self.ego_speed_label.setText(f"{speed_mps * 3.6:.1f}")
            source = (json_data.get('ego_speed') or {}).get('source')
            self.speed_caption.setText(
                f"EGO SPEED km/h · {source}" if source else "EGO SPEED km/h")

    def _update_alarm(self, json_data):
        status = json_data.get('Alarm_status') or {}
        alarm_type = status.get('Alarm_type')

        if alarm_type:
            self._frames_without_alarm = 0
            level = max(1, min(3, int(alarm_type)))
            self.alarm_banner.setText(f"ALARM  LEVEL {level}")
            self.alarm_banner.setProperty("level", str(level))
        else:
            self._frames_without_alarm += 1
            if self._frames_without_alarm >= self.ALARM_CLEAR_FRAMES:
                self.alarm_banner.setText("NO ALARM")
                self.alarm_banner.setProperty("level", "0")
        self._restyle(self.alarm_banner)

    def _update_tables(self, json_data):
        counts = self.scene.counts()
        self.class_table.update_data([
            counts['moving_points'], counts['static_points'],
            counts['moving_tracks'], counts['static_tracks']])
        extent = self.scene.coverage.observed_extent()
        if extent:
            self.coverage_table.update_data(
                [f"{extent[0]:+.0f}", f"{extent[1]:+.0f}", f"{extent[2]:.0f}"])

        static_obj = json_data.get('Static_obj') or {}
        if static_obj:
            self.static_obj_table.update_data([
                self.format_value(static_obj.get('x_val')),
                self.format_value(static_obj.get('y_val')),
            ])

        dynamic_obj = json_data.get('Dynamic_obj') or {}
        if dynamic_obj:
            self.dynamic_obj_table.update_data([
                self.format_value(dynamic_obj.get('x_val')),
                self.format_value(dynamic_obj.get('y_val')),
                self.format_value(dynamic_obj.get('Vx_val')),
                self.format_value(dynamic_obj.get('Vy_val')),
            ])

        now = time.monotonic()
        for key, card_key in (('Dynamic_obj', 'dynamic'),
                              ('Static_obj', 'static'),
                              ('Alarm_thresh', 'thresholds')):
            if json_data.get(key):
                self._last_seen[card_key] = now

        counts = json_data.get('Object_counts') or {}
        if counts:
            self.counts_reported.update_data([
                counts.get('dynamic', '—'), counts.get('static', '—')])

        self._update_object_card(json_data.get('Dynamic_obj') or {})
        self._update_raw_card(json_data)
        self._update_bus_card(json_data.get('Bus') or {})

        thresholds = json_data.get('Alarm_thresh') or {}
        if thresholds:
            # An all-zero set means the unit has no thresholds programmed;
            # showing "0 0 0" reads as real data and hides that.
            levels = [thresholds.get(k) for k in
                      ('Level_1_Sta', 'Level_2_Sta', 'Level_3_Sta',
                       'Level_1_Mov', 'Level_2_Mov', 'Level_3_Mov')]
            if not any(levels):
                self.static_alarm_table.update_data(['not set'] * 3)
                self.dynamic_alarm_table.update_data(['not set'] * 3)
                return

            self.static_alarm_table.update_data([
                self.format_value(thresholds.get('Level_1_Sta'), 0),
                self.format_value(thresholds.get('Level_2_Sta'), 0),
                self.format_value(thresholds.get('Level_3_Sta'), 0),
            ])
            self.dynamic_alarm_table.update_data([
                self.format_value(thresholds.get('Level_1_Mov'), 0),
                self.format_value(thresholds.get('Level_2_Mov'), 0),
                self.format_value(thresholds.get('Level_3_Mov'), 0),
            ])

    def _update_object_card(self, payload):
        """Fill the attribute block, marking derived values as derived."""
        if not payload:
            return
        plot = self.plot_widget
        try:
            dist_x = float(payload.get('x_val'))
            dist_y = float(payload.get('y_val'))
            vrel_x = float(payload.get('Vx_val'))
            vrel_y = float(payload.get('Vy_val'))
        except (TypeError, ValueError):
            return

        heading, basis = plot.object_heading(vrel_x, vrel_y)
        if heading is None:
            orientation = "—"
        else:
            suffix = "derived" if basis == "abs" else "derived, rel"
            orientation = f"{math.degrees(heading):+.0f}°  ({suffix})"

        length, width = plot.object_footprint(dist_y, dist_x)
        from_cluster = any(
            abs(float(c.get('C_X', 0)) - dist_y) < 3.0
            and abs(float(c.get('C_Y', 0)) - dist_x) < 3.0
            for c in plot.clusters)

        at_floor = dist_x <= plot.SENSOR_MIN_RANGE_M
        self.object_card.update_values({
            # Not transmitted by the radar - see the DBC; left blank on purpose
            "objId": None,
            "distX": (f"{dist_x:.2f}  (at sensor minimum)" if at_floor
                      else f"{dist_x:.2f}"),
            "distY": f"{dist_y:.2f}",
            "vrelX": f"{vrel_x:.2f}",
            "vrelY": f"{vrel_y:.2f}",
            "arelX": None,
            "arelY": None,
            "orientation": orientation,
            "dynamic_property": plot.dynamic_property(vrel_x, vrel_y) + " (derived)",
            "classification": None,
            "size": (f"{length:.1f} × {width:.1f}  (cluster)" if from_cluster
                     else f"{length:.1f} × {width:.1f}  (default)"),
        })

    def _update_raw_card(self, json_data):
        """Unscaled counts plus the DBC arithmetic applied to them."""
        payload = json_data.get('Dynamic_obj') or {}
        timing = json_data.get('Time_msg') or {}
        vehicle = json_data.get('Vehicle') or {}

        x_raw = payload.get('x_raw')
        y_raw = payload.get('y_raw')
        values = {
            "x_raw": x_raw,
            "x_check": f"{0.032 * x_raw - 60.8:.3f} m" if x_raw is not None else None,
            "y_raw": y_raw,
            "y_check": f"{0.064 * y_raw - 131.072:.3f} m" if y_raw is not None else None,
            "vx_raw": payload.get('vx_raw'),
            "vy_raw": payload.get('vy_raw'),
            "ttc": timing.get('ttc'),
            "yaw": vehicle.get('yaw_rate'),
        }
        self.raw_card.update_values(values)
        cycle = (json_data.get('Object_counts') or {}).get('dynamic_cycle')
        self._radar_cycle = cycle
        if cycle:
            self._cycle_ever_nonzero = True

    def _update_bus_card(self, bus):
        """Which expected messages are arriving. A group that is never seen is
        a missing feature, not a momentary gap."""
        if not bus:
            return
        values = {}
        for name, entry in bus.items():
            state = entry.get("state")
            if state == "never":
                values[name] = "not sent"
            elif state == "silent":
                values[name] = f"silent {entry.get('age', 0):.0f}s"
            else:
                values[name] = f"{entry.get('hz', 0):.1f} Hz"
        self.bus_card.update_values(values)

    # How long a panel may go without an update before it is called stale.
    STALE_AFTER_S = 1.5

    def _refresh_staleness(self):
        """Age out panels whose message has stopped arriving."""
        now = time.monotonic()
        for key, card in (('dynamic', self.object_card),
                          ('static', self.static_obj_table),
                          ('thresholds', self.dynamic_alarm_table)):
            last = self._last_seen.get(key)
            if last is None:
                continue
            age = now - last
            card.set_stale(age > self.STALE_AFTER_S, age)

    def refresh_status(self):
        """Clock, frame counter and measured frame rate."""
        now = datetime.datetime.now()
        self.timestamp_label.setText(now.strftime("%Y-%m-%d  %H:%M:%S"))

        fps = 0.0
        if len(self._frame_times) >= 2:
            span = self._frame_times[-1] - self._frame_times[0]
            if span > 0:
                fps = (len(self._frame_times) - 1) / span
        self._refresh_staleness()

        # A counter stuck at zero is not implemented on this unit, which is
        # worth distinguishing from one that is genuinely counting.
        cycle = getattr(self, '_radar_cycle', None)
        if cycle in (None, ''):
            cycle_text = "—"
        elif not getattr(self, '_cycle_ever_nonzero', False):
            cycle_text = "not sent"
        else:
            cycle_text = f"{int(cycle):,}"
        self.stream_table.update_data([
            f"{self._frames_seen:,}", f"{fps:.1f}", cycle_text])

    @staticmethod
    def _restyle(widget):
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def closeEvent(self, event):
        if hasattr(self, 'socket_thread'):
            self.socket_thread.stop()
        if hasattr(self, 'status_timer'):
            self.status_timer.stop()
        event.accept()


def build_style_sheet(theme):
    """Qt style sheet for the chrome around the radar scene."""
    t = THEMES[theme]
    return f"""
QMainWindow, QWidget {{ background-color: {t['bg_deep']}; color: {t['text_bright']};
                        font-family: "Helvetica Neue", Arial, sans-serif; }}
#header {{ background-color: {t['header_bg']}; border-bottom: 1px solid {t['panel_border']}; }}
#appTitle {{ font-size: 17px; font-weight: 700; letter-spacing: 2px;
             color: {t['text_bright']}; }}
#caption {{ font-size: 9px; letter-spacing: 1.4px; color: {t['text_dim']}; }}
#egoSpeed {{ font-size: 27px; font-weight: 700; color: {t['text_bright']}; }}
#timestamp {{ font-size: 10px; color: {t['text_dim']}; padding: 2px 2px 6px 2px; }}

#statusChip {{ font-size: 10px; font-weight: 700; letter-spacing: 1.2px;
               padding: 4px 10px; border-radius: 9px; }}
#statusChip[state="on"]  {{ color: {t['alarm_1']};
                            background-color: {rgba(theme, 'alarm_1', 0.12)};
                            border: 1px solid {rgba(theme, 'alarm_1', 0.45)}; }}
#statusChip[state="off"] {{ color: {t['text_dim']};
                            background-color: {rgba(theme, 'text_dim', 0.10)};
                            border: 1px solid {rgba(theme, 'text_dim', 0.35)}; }}

#themeToggle {{ font-size: 10px; font-weight: 700; letter-spacing: 1.2px;
                color: {t['text_mid']}; background-color: {rgba(theme, 'text_dim', 0.10)};
                border: 1px solid {t['panel_border']}; border-radius: 13px; }}
#themeToggle:hover {{ color: {t['text_bright']};
                      background-color: {rgba(theme, 'text_dim', 0.20)}; }}

#alarmBanner {{ font-size: 12px; font-weight: 700; letter-spacing: 1.4px;
                padding: 7px 0; border-radius: 6px; }}
#alarmBanner[level="0"] {{ color: {t['text_dim']};
                           background-color: {rgba(theme, 'text_dim', 0.10)};
                           border: 1px solid {rgba(theme, 'text_dim', 0.30)}; }}
#alarmBanner[level="1"] {{ color: {t['alarm_1']};
                           background-color: {rgba(theme, 'alarm_1', 0.14)};
                           border: 1px solid {rgba(theme, 'alarm_1', 0.55)}; }}
#alarmBanner[level="2"] {{ color: {t['alarm_2']};
                           background-color: {rgba(theme, 'alarm_2', 0.16)};
                           border: 1px solid {rgba(theme, 'alarm_2', 0.60)}; }}
#alarmBanner[level="3"] {{ color: {t['alarm_3']};
                           background-color: {rgba(theme, 'alarm_3', 0.20)};
                           border: 1px solid {rgba(theme, 'alarm_3', 0.75)}; }}

#sidePanel {{ background-color: transparent; }}
#card {{ background-color: {t['panel_bg']}; border: 1px solid {t['panel_border']};
         border-radius: 8px; }}
#cardTitle {{ font-size: 9px; font-weight: 700; letter-spacing: 1.3px;
              color: {t['text_dim']}; padding: 8px 12px 2px 12px; }}

QTableWidget {{ background-color: transparent; border: none;
                color: {t['text_bright']}; font-size: 15px; font-weight: 600; }}
QTableWidget::item {{ padding: 2px; border: none; }}
QHeaderView::section {{ background-color: transparent; color: {t['text_dim']};
                        border: none; font-size: 9px; font-weight: 700;
                        letter-spacing: 0.8px; padding: 0 0 2px 0; }}
#attrName {{ font-size: 11px; color: {t['text_dim']}; }}
#attrValue {{ font-size: 12px; font-weight: 600; color: {t['text_bright']}; }}
#attrValue[missing="yes"] {{ color: {t['text_dim']}; font-weight: 400; }}
#attrValue[stale="yes"] {{ color: {t['text_dim']}; font-weight: 400; }}
#cardTitle[stale="yes"] {{ color: {t['alarm_2']}; }}
QToolTip {{ background-color: {t['panel_bg']}; color: {t['text_bright']};
            border: 1px solid {t['panel_border']}; padding: 4px; }}
QScrollBar:vertical, QScrollBar:horizontal {{ width: 0px; height: 0px; }}
"""


class RadarDataInterface:
    """Interface for receiving JSON data from external sources."""

    def __init__(self, visualizer):
        self.visualizer = visualizer

    def process_frame_data(self, json_data):
        if isinstance(json_data, str):
            json_data = json.loads(json_data)
        self.visualizer.update_with_json(json_data)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Live radar visualizer for the Orion CAN frame processor.")
    parser.add_argument("--theme", choices=sorted(THEMES), default=None,
                        help="Start in this appearance (default: last used)")
    return parser.parse_known_args(argv)[0]


def main():
    args = parse_args(sys.argv[1:])

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    main_window = RadarVisualizerMainWindow(theme=args.theme)
    main_window.show()

    app.radar_interface = RadarDataInterface(main_window)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
