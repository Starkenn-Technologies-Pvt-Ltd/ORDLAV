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

        # Scene extents in metres
        self.half_width = 24.0     # lateral, each side of boresight
        self.max_range = 120.0     # longitudinal
        self.lateral_step = 6.0
        self.range_step = 10.0

        self.frame = {}
        self.detected_points = []
        self.tracks = []
        self.clusters = []
        self.static_obj = None
        self.dynamic_obj = None
        self.alarm_thresholds = None
        self.alarm_type = None

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

    def to_pixel_x(self, lateral_m):
        rect = self.plot_rect()
        ratio = (lateral_m + self.half_width) / (2.0 * self.half_width)
        return rect.left() + ratio * rect.width()

    def to_pixel_y(self, range_m):
        rect = self.plot_rect()
        return rect.bottom() - (range_m / self.max_range) * rect.height()

    def metres_to_pixels_x(self, metres):
        return (metres / (2.0 * self.half_width)) * self.plot_rect().width()

    def metres_to_pixels_y(self, metres):
        return (metres / self.max_range) * self.plot_rect().height()

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

        # Lateral grid, spanning the full plot height
        lateral = -self.half_width
        while lateral <= self.half_width + 1e-6:
            x = self.to_pixel_x(lateral)
            is_major = abs(lateral) < 1e-6
            painter.setPen(QPen(self.c('axis_line') if is_major else self.c('grid_minor'),
                                2 if is_major else 1))
            painter.drawLine(int(x), int(rect.top()), int(x), int(rect.bottom()))
            painter.setPen(QPen(self.c('text_mid') if is_major else self.c('text_dim')))
            painter.drawText(QRectF(x - 24, rect.bottom() + 6, 48, 16),
                             Qt.AlignCenter, f"{lateral:g}")
            lateral += self.lateral_step

        # Range grid, with the top of the scene always labelled
        range_m = 0.0
        ticks = []
        while range_m <= self.max_range + 1e-6:
            ticks.append(range_m)
            range_m += self.range_step
        if ticks and abs(ticks[-1] - self.max_range) > self.range_step * 0.3:
            ticks.append(self.max_range)

        for range_m in ticks:
            y = self.to_pixel_y(range_m)
            is_major = abs(range_m % (self.range_step * 5)) < 1e-6
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
        key = (self.width(), self.height(), self.half_width,
               self.max_range, self.theme)
        if self._grid_cache is None or self._grid_key != key:
            self._grid_cache = self._build_grid()
            self._grid_key = key

        painter = QPainter(self)
        painter.drawPixmap(0, 0, self._grid_cache)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setClipRect(self.plot_rect())

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

            halo = QColor(self.c('track'))
            halo.setAlpha(38)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(halo))
            painter.drawEllipse(centre, 15, 15)

            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(self.c('track'), 1.8))
            painter.drawRoundedRect(QRectF(centre.x() - 8, centre.y() - 8, 16, 16), 3, 3)
            painter.setBrush(QBrush(self.c('track')))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(centre, 2.4, 2.4)

            self._draw_velocity_arrow(painter, centre, v_lateral, v_range, self.c('track'))

            speed = (v_lateral ** 2 + v_range ** 2) ** 0.5
            label = f"T{index + 1}  {range_m:.0f} m  {speed:.1f} m/s"
            self._draw_label(painter, label, centre.x() + 13, centre.y() - 20, self.c('track'))

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
        glow.setAlpha(45)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(glow))
        painter.drawEllipse(centre, 19, 19)

        painter.setBrush(QBrush(colour))
        painter.setPen(QPen(self.c('marker_edge'), 1.2))
        if square:
            painter.drawRect(QRectF(centre.x() - 6, centre.y() - 6, 12, 12))
        else:
            painter.drawEllipse(centre, 6.5, 6.5)

        if not square:
            self._draw_velocity_arrow(painter, centre, obj['vy'], obj['vx'], colour)

        label = f"{tag}  {range_m:.1f} m / {lateral:+.1f} m"
        self._draw_label(painter, label, centre.x() + 12, centre.y() + 8, colour)

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
            (self.c('cluster'), "cluster"),
            (self.c('track'), "track"),
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
        self.half_width = max(6.0, min(80.0, self.half_width * factor))
        self.range_step = 10.0 if self.max_range <= 160 else 25.0
        self.lateral_step = 6.0 if self.half_width <= 30 else 10.0
        self._grid_cache = None
        self.update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._grid_cache = None


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

    def set_accent(self, colour):
        for item in self.table_items:
            item.setForeground(QBrush(colour))


class RadarVisualizerMainWindow(QMainWindow):
    """Main window: radar scene on the left, readouts on the right."""

    # Frames without an alarm message before the banner clears
    ALARM_CLEAR_FRAMES = 4

    def __init__(self, theme=None):
        super(RadarVisualizerMainWindow, self).__init__()
        self.setWindowTitle("Orion Radar Visualizer")
        self.setGeometry(100, 100, 1500, 880)

        self.settings = QSettings("Starkenn", "OrionRadarVisualizer")
        stored = self.settings.value("theme", DEFAULT_THEME)
        self.theme = theme or (stored if stored in THEMES else DEFAULT_THEME)

        self._frame_times = deque(maxlen=40)
        self._frames_seen = 0
        self._frames_without_alarm = 0
        self._connected = False

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
        self.theme_button.setText("LIGHT" if theme == 'dark' else "DARK")
        self._apply_table_accents()

        for widget in (self.connection_label, self.alarm_banner, self.theme_button):
            self._restyle(widget)

    def _apply_table_accents(self):
        self.static_alarm_table.set_accent(self.c('text_mid'))
        self.dynamic_alarm_table.set_accent(self.c('text_mid'))
        self.static_obj_table.set_accent(self.c('static_ooi'))
        self.dynamic_obj_table.set_accent(self.c('dynamic_ooi'))
        for card in (self.counts_table, self.stream_table):
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

        self.plot_widget = RadarPlotWidget(theme=self.theme)
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
        speed_caption = QLabel("EGO SPEED km/h")
        speed_caption.setObjectName("caption")
        speed_caption.setAlignment(Qt.AlignRight | Qt.AlignTop)
        speed_layout.addWidget(self.ego_speed_label)
        speed_layout.addWidget(speed_caption)
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

        self.counts_table = DataTableWidget("Scene", ["POINTS", "CLUSTERS", "TRACKS"])
        self.static_obj_table = DataTableWidget(
            "Static object of interest", ["RANGE m", "LATERAL m"])
        self.dynamic_obj_table = DataTableWidget(
            "Moving object of interest", ["RANGE m", "LATERAL m", "Vx m/s", "Vy m/s"])
        self.static_alarm_table = DataTableWidget(
            "Static alarm thresholds", ["L1 m", "L2 m", "L3 m"])
        self.dynamic_alarm_table = DataTableWidget(
            "Moving alarm thresholds", ["L1 m", "L2 m", "L3 m"])
        self.stream_table = DataTableWidget("Stream", ["FRAMES", "FPS"])

        for card in (self.counts_table, self.static_obj_table, self.dynamic_obj_table,
                     self.static_alarm_table, self.dynamic_alarm_table, self.stream_table):
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

            self.plot_widget.update_data(json_data)
            self._update_ego_speed(json_data)
            self._update_alarm(json_data)
            self._update_tables(json_data)
        except Exception as e:
            print(f"Error updating display: {e}")

    def _update_ego_speed(self, json_data):
        try:
            speed = float((json_data.get('ego_speed') or {}).get('ego_speed'))
        except (TypeError, ValueError, AttributeError):
            return  # keep the last known speed rather than blanking it
        if speed >= 0:
            self.ego_speed_label.setText(f"{speed:.1f}")

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
        plot = self.plot_widget
        self.counts_table.update_data([
            len(plot.detected_points), len(plot.clusters), len(plot.tracks)])

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

        thresholds = json_data.get('Alarm_thresh') or {}
        if thresholds:
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

    def refresh_status(self):
        """Clock, frame counter and measured frame rate."""
        now = datetime.datetime.now()
        self.timestamp_label.setText(now.strftime("%Y-%m-%d  %H:%M:%S"))

        fps = 0.0
        if len(self._frame_times) >= 2:
            span = self._frame_times[-1] - self._frame_times[0]
            if span > 0:
                fps = (len(self._frame_times) - 1) / span
        self.stream_table.update_data([f"{self._frames_seen:,}", f"{fps:.1f}"])

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
