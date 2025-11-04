import json
import sys
import socket
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QTableWidget, QTableWidgetItem, QLabel,
                             QFrame, QSizePolicy, QHeaderView)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QColor, QPainter, QPen, QBrush, QPixmap
import numpy as np
import time
import datetime

HOST = '127.0.0.1'  # CAN Processor IP
PORT = 5001  # CAN Processor Port
count = 0

class SocketClientThread(QThread):
    """Optimized background thread for handling socket communication."""
    data_received = pyqtSignal(object)  # Signal to send received data to UI

    def __init__(self):
        super().__init__()
        self.running = True
        self.buffer = ""  # Buffer to accumulate incoming data

    def run(self):
        """Connects to CAN processor server, receives JSON data, and emits it to UI."""
        while self.running:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                    client_socket.connect((HOST, PORT))
                    while self.running:
                        data = client_socket.recv(1024).decode('utf-8')
                        if data:
                            self.buffer += data
                            # Remove ping messages efficiently
                            if 'ping' in self.buffer:
                                self.buffer = self.buffer.replace('ping', '')
                            # Process complete JSON messages
                            while "\n" in self.buffer:
                                message, self.buffer = self.buffer.split("\n", 1)
                                message = message.strip()
                                if message and (message.startswith('{') or message.startswith('[')):
                                    try:
                                        json_data = json.loads(message)
                                        self.data_received.emit(json_data)
                                    except json.JSONDecodeError:
                                        # Try to recover JSON without print
                                        json_start = message.find('{')
                                        if json_start != -1:
                                            try:
                                                potential_json = message[json_start:]
                                                json_data = json.loads(potential_json)
                                                self.data_received.emit(json_data)
                                            except json.JSONDecodeError:
                                                pass  # Silent fail for performance
                        else:
                            break  # Connection closed
            except ConnectionRefusedError:
                self.sleep(3)
            except Exception as e:
                print(f"❌ Socket Error: {e}")
                self.sleep(3)

    def stop(self):
        """Stops the thread safely."""
        self.running = False
        self.quit()
        self.wait()

class RadarPlotWidget(QWidget):
    """Optimized PyQt radar visualization widget with object pooling"""

    def __init__(self, parent=None):
        super(RadarPlotWidget, self).__init__(parent)
        self.setMinimumSize(800, 600)
        self.setStyleSheet("background-color: black;")

        # Radar parameters (meters)
        self.min_x, self.max_x = -20, 20
        self.min_y, self.max_y = 0, 30
        self.grid_step = 3

        # Object pools for widget reuse (MAJOR OPTIMIZATION)
        self.point_pool = []  # Pool of reusable point widgets
        self.track_pool = []  # Pool of reusable track widgets
        self.static_pool = []  # Pool for static objects
        self.dynamic_pool = []  # Pool for dynamic objects
        self.text_pool = []  # Pool for text labels

        # Pool sizes
        self.max_points = 200
        self.max_tracks = 32
        self.max_static = 1
        self.max_dynamic = 1
        self.max_text = 20

        # Initialize pools
        self._initialize_pools()

        # Active widget counters
        self.active_points = 0
        self.active_tracks = 0
        self.active_static = 0
        self.active_dynamic = 0
        self.active_text = 0

        # Background grid (drawn once)
        self.background_pixmap = None
        self.create_background()

    def _initialize_pools(self):
        """Initialize widget pools for maximum performance"""
        # Create point widgets pool
        for _ in range(self.max_points):
            label = QLabel(self)
            label.hide()
            self.point_pool.append(label)

        # Create track widgets pool
        for _ in range(self.max_tracks):
            label = QLabel(self)
            label.hide()
            self.track_pool.append(label)

        # Create static object widgets pool
        for _ in range(self.max_static):
            label = QLabel(self)
            label.hide()
            self.static_pool.append(label)

        # Create dynamic object widgets pool
        for _ in range(self.max_dynamic):
            label = QLabel(self)
            label.hide()
            self.dynamic_pool.append(label)

        # Create text widgets pool
        for _ in range(self.max_text):
            label = QLabel(self)
            label.hide()
            self.text_pool.append(label)

    def create_background(self):
        """Create radar grid background with 3m step size"""
        self.background_pixmap = QPixmap(self.size())
        self.background_pixmap.fill(Qt.black)

        painter = QPainter(self.background_pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        # Grid pen for 3m steps
        grid_pen = QPen(QColor(60, 60, 60), 1, Qt.SolidLine)
        painter.setPen(grid_pen)

        # Draw vertical grid lines every 3m
        for x in range(-18, 19, self.grid_step):
            x_pixel = self.normalize_x(x)
            painter.drawLine(x_pixel, 0, x_pixel, self.height())
            # Add X-axis labels every 3m
            if x % 3 == 0:
                painter.setPen(QPen(Qt.white))
                painter.setFont(QFont("Arial", 8))
                painter.drawText(x_pixel - 8, self.height() - 10, f"{x}")
                painter.setPen(grid_pen)

        # Draw horizontal grid lines every 3m
        for y in range(0, int(self.max_y) + 1, self.grid_step):
            y_pixel = self.normalize_y(y)
            painter.drawLine(0, y_pixel, self.width(), y_pixel)
            # Add Y-axis labels every 3m
            if y % 3 == 0 and y != 0:
                painter.setPen(QPen(Qt.white))
                painter.setFont(QFont("Arial", 8))
                painter.drawText(self.width() // 2 - 25, y_pixel + 4, f"{y}")
                painter.setPen(grid_pen)

        # Draw main axes
        axis_pen = QPen(QColor(120, 120, 120), 2)
        painter.setPen(axis_pen)
        painter.drawLine(self.width() // 2, 0, self.width() // 2, self.height())
        painter.drawLine(0, self.height(), self.width(), self.height())

        # Add axis labels
        text_pen = QPen(Qt.white)
        painter.setPen(text_pen)
        painter.setFont(QFont("Arial", 12, QFont.Bold))
        painter.drawText(self.width() // 2 + 10, 20, "Range (m)")
        painter.drawText(self.width() - 90, self.height() - 35, "Azimuth (m)")

        painter.end()

    def normalize_x(self, x):
        """Convert radar X coordinate to pixel X"""
        return int(((x - self.min_x) / (self.max_x - self.min_x)) * self.width())

    def normalize_y(self, y):
        """Convert radar Y coordinate to pixel Y"""
        return int(self.height() - (y / self.max_y) * self.height())

    def _get_widget_from_pool(self, pool, pool_counter, max_count):
        """Get a widget from the specified pool"""
        if pool_counter < max_count and pool_counter < len(pool):
            return pool[pool_counter]
        return None

    def _setup_point_widget(self, widget, x_pixel, y_pixel, point_type="detected"):
        """Setup a widget for a specific point type"""
        if point_type == "detected":
            style = "background-color: cyan; border-radius: 3px;"
            size = (8, 8)
        elif point_type == "tracked":
            style = "background-color: blue; border: 2px solid blue; border-radius: 4px;"
            size = (12, 12)
        elif point_type == "static_interest":
            style = "background-color: red; border: 3px solid red; border-radius: 6px;"
            size = (16, 16)
        elif point_type == "dynamic_interest":
            style = "background-color: red; border: 3px solid red; border-radius: 6px;"
            size = (16, 16)
        else:
            style = "background-color: white; border-radius: 2px;"
            size = (6, 6)

        widget.setStyleSheet(style)
        widget.resize(*size)
        widget.move(x_pixel - size[0]//2, y_pixel - size[1]//2)
        widget.show()

    def update_data(self, json_data):
        """Optimized update with widget pooling - NO widget creation/deletion"""
        try:
            # Reset active counters
            self.active_points = 0
            self.active_tracks = 0
            self.active_static = 0
            self.active_dynamic = 0
            self.active_text = 0

            # Hide all widgets first (faster than deletion)
            self._hide_all_widgets()

            detected_points = []
            tracked_objects = []

            # Extract detected points
            if 'Point_cloud' in json_data and 'Detect_Points' in json_data['Point_cloud']:
                detected_points = json_data['Point_cloud']['Detect_Points']

            # Extract tracked objects
            if 'Tracked_obj' in json_data and 'Track_Objects' in json_data['Tracked_obj']:
                tracked_objects = json_data['Tracked_obj']['Track_Objects']

            # Process detected points using pool
            for i, point in enumerate(detected_points[:self.max_points]):
                widget = self._get_widget_from_pool(self.point_pool, i, self.max_points)
                if widget:
                    x = float(point.get('D_X', 0)) * -1
                    y = float(point.get('D_Y', 0))
                    x_pixel = self.normalize_x(x)
                    y_pixel = self.normalize_y(y)
                    self._setup_point_widget(widget, x_pixel, y_pixel, "detected")
                    self.active_points += 1

            # Process tracked objects using pool
            for i, track in enumerate(tracked_objects[:self.max_tracks]):
                widget = self._get_widget_from_pool(self.track_pool, i, self.max_tracks)
                if widget:
                    x = float(track.get('T_X', 0)) * -1
                    y = float(track.get('T_Y', 0))
                    x_pixel = self.normalize_x(x)
                    y_pixel = self.normalize_y(y)
                    self._setup_point_widget(widget, x_pixel, y_pixel, "tracked")
                    self.active_tracks += 1

            # Process static object of interest
            if 'Static_obj' in json_data and json_data['Static_obj']:
                static_obj = json_data['Static_obj']
                if 'x_val' in static_obj and 'y_val' in static_obj:
                    widget = self._get_widget_from_pool(self.static_pool, 0, self.max_static)
                    text_widget = self._get_widget_from_pool(self.text_pool, self.active_text, self.max_text)
                    if widget:
                        y = float(static_obj.get('x_val', 0))
                        x = float(static_obj.get('y_val', 0))
                        x_pixel = self.normalize_x(x)
                        y_pixel = self.normalize_y(y)
                        self._setup_point_widget(widget, x_pixel, y_pixel, "static_interest")
                        self.active_static += 1

                        if text_widget:
                            text_widget.setText(f"({x:.1f}, {y:.1f})")
                            text_widget.setStyleSheet("""
                                color: white;
                                background-color: rgba(0,0,0,180);
                                padding: 3px 6px;
                                border-radius: 4px;
                                font: bold 8px Arial;
                            """)
                            text_widget.adjustSize()
                            text_widget.move(x_pixel + 8, y_pixel - 18)
                            text_widget.show()
                            self.active_text += 1

            # Process dynamic object of interest
            if 'Dynamic_obj' in json_data and json_data['Dynamic_obj']:
                dynamic_obj = json_data['Dynamic_obj']
                if 'x_val' in dynamic_obj and 'y_val' in dynamic_obj:
                    widget = self._get_widget_from_pool(self.dynamic_pool, 0, self.max_dynamic)
                    text_widget = self._get_widget_from_pool(self.text_pool, self.active_text, self.max_text)
                    if widget:
                        y = float(dynamic_obj.get('x_val', 0))
                        x = float(dynamic_obj.get('y_val', 0))
                        x_pixel = self.normalize_x(x)
                        y_pixel = self.normalize_y(y)
                        self._setup_point_widget(widget, x_pixel, y_pixel, "dynamic_interest")
                        self.active_dynamic += 1

                        if text_widget:
                            text_widget.setText(f"({x:.1f}, {y:.1f})")
                            text_widget.setStyleSheet("""
                                color: white;
                                background-color: rgba(0,0,0,180);
                                padding: 3px 6px;
                                border-radius: 4px;
                                font: bold 8px Arial;
                            """)
                            text_widget.adjustSize()
                            text_widget.move(x_pixel + 8, y_pixel - 18)
                            text_widget.show()
                            self.active_text += 1

        except Exception as e:
            print(f"❌ Error updating radar plot: {e}")

    def _hide_all_widgets(self):
        """Efficiently hide all widgets instead of deleting them"""
        # Hide point widgets
        for i in range(self.active_points, len(self.point_pool)):
            self.point_pool[i].hide()

        # Hide track widgets
        for i in range(self.active_tracks, len(self.track_pool)):
            self.track_pool[i].hide()

        # Hide static widgets
        for i in range(self.active_static, len(self.static_pool)):
            self.static_pool[i].hide()

        # Hide dynamic widgets
        for i in range(self.active_dynamic, len(self.dynamic_pool)):
            self.dynamic_pool[i].hide()

        # Hide text widgets
        for i in range(self.active_text, len(self.text_pool)):
            self.text_pool[i].hide()

    def paintEvent(self, event):
        """Paint the background grid"""
        if self.background_pixmap:
            painter = QPainter(self)
            painter.drawPixmap(0, 0, self.background_pixmap)

    def resizeEvent(self, event):
        """Handle window resize"""
        super().resizeEvent(event)
        self.create_background()
        self.update()

class DataTableWidget(QFrame):
    """Optimized table widget for displaying radar data"""

    def __init__(self, title, headers, parent=None):
        super(DataTableWidget, self).__init__(parent)
        self.setFrameStyle(QFrame.Box)
        self.setLineWidth(1)

        layout = QVBoxLayout()

        # Title label
        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setFont(QFont("Arial", 10, QFont.Bold))
        title_label.setStyleSheet("color: white; background-color: #2e2e2e; padding: 5px;")
        layout.addWidget(title_label)

        # Table
        self.table = QTableWidget(1, len(headers))
        self.table.setHorizontalHeaderLabels(headers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setMaximumHeight(80)
        self.table.setStyleSheet("""
            QTableWidget {
                background-color: #1e1e1e;
                color: white;
                gridline-color: #3e3e3e;
                border: 1px solid #3e3e3e;
            }
            QHeaderView::section {
                background-color: #2e2e2e;
                color: white;
                border: 1px solid #3e3e3e;
                padding: 4px;
            }
        """)
        layout.addWidget(self.table)
        self.setLayout(layout)

        # Pre-create table items for reuse
        self.table_items = []
        for col in range(len(headers)):
            item = QTableWidgetItem("")
            item.setTextAlignment(Qt.AlignCenter)
            self.table.setItem(0, col, item)
            self.table_items.append(item)

    def update_data(self, data_list):
        """Optimized table update - reuse existing items"""
        for col, value in enumerate(data_list):
            if col < len(self.table_items):
                self.table_items[col].setText(str(value))

class RadarVisualizerMainWindow(QMainWindow):
    def format_value(self, value, decimal_places=1):
        try:
            if value == 'N/A' or value is None:
                return 'N/A'
            return f"{float(value):.{decimal_places}f}"
        except (ValueError, TypeError):
            return str(value)

    def __init__(self):
        super(RadarVisualizerMainWindow, self).__init__()
        self.setWindowTitle("Real-Time Radar Visualizer - Optimized High Performance")
        self.setGeometry(100, 100, 1400, 800)

        # Set dark theme
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
                color: white;
            }
        """)

        self.setup_ui()

        # Start socket communication in a QThread
        self.socket_thread = SocketClientThread()
        self.socket_thread.data_received.connect(self.update_with_json)
        self.socket_thread.start()

        # Setup independent timestamp timer (updates every 100ms for smooth display)
        self.timestamp_timer = QTimer()
        self.timestamp_timer.timeout.connect(self.update_timestamp)
        self.timestamp_timer.start(100)  # Update every 100ms

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main horizontal layout
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)

        # Left side - Optimized Radar Plot (75% width)
        self.plot_widget = RadarPlotWidget()
        main_layout.addWidget(self.plot_widget, 75)

        # Right side - Data tables (25% width)
        right_panel = QWidget()
        right_panel.setMaximumWidth(300)
        right_layout = QVBoxLayout()
        right_panel.setLayout(right_layout)

        # Create data tables
        self.static_obj_table = DataTableWidget("Static Object of Interest", ["X (m)", "Y (m)"])
        self.dynamic_obj_table = DataTableWidget("Dynamic Object of Interest", ["X (m)", "Y (m)", "Vx (m/s)", "Vy (m/s)"])
        self.static_alarm_table = DataTableWidget("Static Alarm Threshold", ["L1", "L2", "L3"])
        self.dynamic_alarm_table = DataTableWidget("Dynamic Alarm Threshold", ["L1", "L2", "L3"])
        self.alarm_status_table = DataTableWidget("Alarm Status", ["Alarms"])

        # Add tables to right panel
        right_layout.addWidget(self.static_obj_table)
        right_layout.addWidget(self.dynamic_obj_table)
        right_layout.addWidget(self.static_alarm_table)
        right_layout.addWidget(self.dynamic_alarm_table)
        right_layout.addWidget(self.alarm_status_table)
        right_layout.addStretch()

        main_layout.addWidget(right_panel, 25)

        # Create ego speed label (top left corner)
        self.ego_speed_label = QLabel("Ego Speed: N/A km/h", self)
        self.ego_speed_label.setStyleSheet("""
            QLabel {
                color: white;
                background-color: rgba(0, 0, 0, 180);
                padding: 8px 12px;
                font: bold 12px Arial;
            }
        """)
        self.ego_speed_label.adjustSize()
        self.ego_speed_label.move(self.width() - 500, 10)  # Top left corner

        # Create timestamp label (bottom left corner)
        self.timestamp_label = QLabel("Timestamp: N/A", self)
        self.timestamp_label.setStyleSheet("""
            QLabel {
                color: white;
                background-color: rgba(0, 0, 0, 180);
                padding: 6px 10px;
                font: 10px Arial;
            }
        """)
        self.timestamp_label.adjustSize()
        # Position will be set in resizeEvent

    def update_timestamp(self):
        """Update timestamp display independently - called by timer"""
        try:
            current_timestamp = time.time()
            # Convert UNIX timestamp to readable format
            dt = datetime.datetime.fromtimestamp(current_timestamp)
            formatted_time = dt.strftime("%Y-%m-%d %H:%M:%S")
            unix_time_str = f"{current_timestamp:.0f}"
            self.timestamp_label.setText(f"Time: {formatted_time} (Unix: {unix_time_str})")
            self.timestamp_label.adjustSize()
        except (ValueError, TypeError, OSError):
            self.timestamp_label.setText(f"Timestamp: {time.time()}")
            self.timestamp_label.adjustSize()

    def update_with_json(self, json_data):
        """Optimized update method with minimal operations"""
        try:
            # Update radar plot (OPTIMIZED - uses widget pooling)
            self.plot_widget.update_data(json_data)

            # Update ego speed display
            ego_speed = json_data.get('ego_speed', 'N/A')
            speed_value = float(ego_speed['ego_speed'])
            print(speed_value)
            if speed_value >= 0:
                try:
                    # Convert to float and format
                    #speed_value = float(ego_speed)
                    #print("Hi")
                    self.ego_speed_label.setText(f"Ego Speed: {speed_value:.1f} km/h")
                except (ValueError, TypeError):
                    self.ego_speed_label.setText(f"Ego Speed: N/A km/h")
            else:
                self.ego_speed_label.setText("Ego Speed: N/A km/h")
            self.ego_speed_label.adjustSize()

            # NOTE: Timestamp is now updated independently by timer, not here

            # Batch table updates
            self._update_tables(json_data)

        except Exception as e:
            print(f"❌ Error updating display: {e}")

    def _update_tables(self, json_data):
        """Optimized table updates"""
        # Update static object table
        if 'Static_obj' in json_data and json_data['Static_obj']:
            static_data = [
                self.format_value(json_data['Static_obj'].get('x_val', 'N/A')),
                self.format_value(json_data['Static_obj'].get('y_val', 'N/A'))
            ]
            self.static_obj_table.update_data(static_data)

        # Update dynamic object table
        if 'Dynamic_obj' in json_data and json_data['Dynamic_obj']:
            dynamic_data = [
                self.format_value(json_data['Dynamic_obj'].get('x_val', 'N/A')),
                self.format_value(json_data['Dynamic_obj'].get('y_val', 'N/A')),
                self.format_value(json_data['Dynamic_obj'].get('Vx_val', 'N/A')),
                self.format_value(json_data['Dynamic_obj'].get('Vy_val', 'N/A'))
            ]
            self.dynamic_obj_table.update_data(dynamic_data)

        # Update alarm thresholds
        if 'Alarm_thresh' in json_data and json_data['Alarm_thresh']:
            static_alarm_data = [
                self.format_value(json_data['Alarm_thresh'].get('Level_1_Sta', 'N/A')),
                self.format_value(json_data['Alarm_thresh'].get('Level_2_Sta', 'N/A')),
                self.format_value(json_data['Alarm_thresh'].get('Level_3_Sta', 'N/A'))
            ]
            self.static_alarm_table.update_data(static_alarm_data)

            dynamic_alarm_data = [
                self.format_value(json_data['Alarm_thresh'].get('Level_1_Mov', 'N/A')),
                self.format_value(json_data['Alarm_thresh'].get('Level_2_Mov', 'N/A')),
                self.format_value(json_data['Alarm_thresh'].get('Level_3_Mov', 'N/A'))
            ]
            self.dynamic_alarm_table.update_data(dynamic_alarm_data)

        global count
        # Update alarm status
        if 'Alarm_status' in json_data and json_data['Alarm_status']:
            alarm_type = json_data['Alarm_status'].get('Alarm_type', 0)
            alarm_text = f"Alarm {alarm_type}"
            self.alarm_status_table.update_data([alarm_text])
            count = 0
        else:
            if count > 3:
                alarm_text = "No Alarm"
                self.alarm_status_table.update_data([alarm_text])
                count = 0
            count += 1

    def resizeEvent(self, event):
        """Handle window resize and reposition labels"""
        super().resizeEvent(event)

        # Reposition timestamp label to bottom left
        self.timestamp_label.move(10, self.height() - self.timestamp_label.height() - 30)

    def closeEvent(self, event):
        """Handle application close event"""
        if hasattr(self, 'socket_thread'):
            self.socket_thread.stop()
        if hasattr(self, 'timestamp_timer'):
            self.timestamp_timer.stop()
        event.accept()

# Data input interface for integration with CAN processor
class RadarDataInterface:
    """Interface for receiving JSON data from external sources"""

    def __init__(self, visualizer):
        self.visualizer = visualizer

    def process_frame_data(self, json_data):
        """Process incoming frame data"""
        if isinstance(json_data, str):
            json_data = json.loads(json_data)
        self.visualizer.update_with_json(json_data)

def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # Create and show main window
    main_window = RadarVisualizerMainWindow()
    main_window.show()

    # Create data interface for external integration
    data_interface = RadarDataInterface(main_window)
    app.radar_interface = data_interface

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
