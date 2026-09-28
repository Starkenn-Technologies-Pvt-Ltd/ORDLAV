import json
import sys
import socket
import time
import datetime

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QTableWidget,
    QTableWidgetItem,
    QLabel,
    QFrame,
    QHeaderView
)

from PyQt5.QtCore import (
    QTimer,
    Qt,
    pyqtSignal,
    QThread
)

from PyQt5.QtGui import (
    QFont,
    QColor,
    QPainter,
    QPen,
    QPixmap
)


# ============================================================
# CONFIGURATION
# ============================================================

HOST = '127.0.0.1'
PORT = 5001

count = 0


# ============================================================
# SOCKET CLIENT THREAD
# ============================================================

class SocketClientThread(QThread):
    """
    Background thread for receiving JSON CAN data over TCP.
    """

    data_received = pyqtSignal(object)

    def __init__(self):
        super().__init__()

        self.running = True
        self.buffer = ""

    def run(self):
        """
        Connect to CAN processor and continuously receive JSON data.
        """

        while self.running:

            try:

                with socket.socket(
                    socket.AF_INET,
                    socket.SOCK_STREAM
                ) as client_socket:

                    client_socket.settimeout(1.0)

                    client_socket.connect(
                        (HOST, PORT)
                    )

                    while self.running:

                        try:
                            data = client_socket.recv(4096)

                        except socket.timeout:
                            continue

                        if not data:
                            break

                        try:
                            decoded_data = data.decode(
                                'utf-8',
                                errors='ignore'
                            )

                        except Exception:
                            continue

                        self.buffer += decoded_data

                        # ------------------------------------------------
                        # Remove ping messages
                        # ------------------------------------------------

                        self.buffer = self.buffer.replace(
                            'ping',
                            ''
                        )

                        # ------------------------------------------------
                        # Process complete JSON messages
                        #
                        # Assumption:
                        # Each JSON message ends with '\n'
                        # ------------------------------------------------

                        while "\n" in self.buffer:

                            message, self.buffer = self.buffer.split(
                                "\n",
                                1
                            )

                            message = message.strip()

                            if not message:
                                continue

                            if not (
                                message.startswith("{")
                                or
                                message.startswith("[")
                            ):
                                continue

                            try:

                                json_data = json.loads(
                                    message
                                )

                                self.data_received.emit(
                                    json_data
                                )

                            except json.JSONDecodeError:

                                # ----------------------------------------
                                # Attempt JSON recovery
                                # ----------------------------------------

                                json_start = message.find("{")

                                if json_start != -1:

                                    try:

                                        potential_json = (
                                            message[json_start:]
                                        )

                                        json_data = json.loads(
                                            potential_json
                                        )

                                        self.data_received.emit(
                                            json_data
                                        )

                                    except json.JSONDecodeError:
                                        pass

            except ConnectionRefusedError:

                self.msleep(3000)

            except Exception as e:

                print(
                    f"Socket Error: {e}"
                )

                self.msleep(3000)

    def stop(self):
        """
        Stop socket thread safely.
        """

        self.running = False

        self.quit()

        self.wait(2000)


# ============================================================
# RADAR PLOT WIDGET
# ============================================================

class RadarPlotWidget(QWidget):
    """
    Radar visualization widget.

    Uses reusable QLabel objects instead of creating/deleting
    widgets for every incoming CAN frame.
    """

    def __init__(self, parent=None):

        super().__init__(parent)

        self.setMinimumSize(
            800,
            600
        )

        self.setStyleSheet(
            "background-color: black;"
        )

        # --------------------------------------------------------
        # Radar coordinates
        # --------------------------------------------------------

        self.min_x = -50
        self.max_x = 50

        self.min_y = 0
        self.max_y = 150

        self.grid_step = 3

        # --------------------------------------------------------
        # Widget pools
        # --------------------------------------------------------

        self.point_pool = []
        self.track_pool = []
        self.static_pool = []
        self.dynamic_pool = []
        self.text_pool = []
        self.cluster_pool = []

        # --------------------------------------------------------
        # Maximum objects
        # --------------------------------------------------------

        self.max_points = 100
        self.max_tracks = 32
        self.max_static = 1
        self.max_dynamic = 1
        self.max_text = 20
        self.max_clusters = 64

        self._initialize_pools()

        # --------------------------------------------------------
        # Active counters
        # --------------------------------------------------------

        self.active_points = 0
        self.active_tracks = 0
        self.active_static = 0
        self.active_dynamic = 0
        self.active_text = 0
        self.active_clusters = 0

        # --------------------------------------------------------
        # Background
        # --------------------------------------------------------

        self.background_pixmap = None

        self.create_background()

    # ========================================================
    # POOL INITIALIZATION
    # ========================================================

    def _initialize_pools(self):

        for _ in range(self.max_points):

            label = QLabel(self)

            label.hide()

            self.point_pool.append(
                label
            )

        for _ in range(self.max_tracks):

            label = QLabel(self)

            label.hide()

            self.track_pool.append(
                label
            )

        for _ in range(self.max_clusters):

            label = QLabel(self)

            label.hide()

            self.cluster_pool.append(
                label
            )

        for _ in range(self.max_static):

            label = QLabel(self)

            label.hide()

            self.static_pool.append(
                label
            )

        for _ in range(self.max_dynamic):

            label = QLabel(self)

            label.hide()

            self.dynamic_pool.append(
                label
            )

        for _ in range(self.max_text):

            label = QLabel(self)

            label.hide()

            self.text_pool.append(
                label
            )

    # ========================================================
    # BACKGROUND GRID
    # ========================================================

    def create_background(self):

        self.background_pixmap = QPixmap(
            self.size()
        )

        self.background_pixmap.fill(
            Qt.black
        )

        painter = QPainter(
            self.background_pixmap
        )

        painter.setRenderHint(
            QPainter.Antialiasing
        )

        grid_pen = QPen(
            QColor(60, 60, 60),
            1,
            Qt.SolidLine
        )

        painter.setPen(
            grid_pen
        )

        # ----------------------------------------------------
        # Vertical grid
        # ----------------------------------------------------

        for x in range(
            -18,
            19,
            self.grid_step
        ):

            x_pixel = self.normalize_x(x)

            painter.drawLine(
                x_pixel,
                0,
                x_pixel,
                self.height()
            )

            painter.setPen(
                QPen(Qt.white)
            )

            painter.setFont(
                QFont(
                    "Arial",
                    8
                )
            )

            painter.drawText(
                x_pixel - 8,
                self.height() - 10,
                f"{x}"
            )

            painter.setPen(
                grid_pen
            )

        # ----------------------------------------------------
        # Horizontal grid
        # ----------------------------------------------------

        for y in range(
            0,
            int(self.max_y) + 1,
            self.grid_step
        ):

            y_pixel = self.normalize_y(y)

            painter.drawLine(
                0,
                y_pixel,
                self.width(),
                y_pixel
            )

            if y != 0:

                painter.setPen(
                    QPen(Qt.white)
                )

                painter.setFont(
                    QFont(
                        "Arial",
                        8
                    )
                )

                painter.drawText(
                    self.width() // 2 - 25,
                    y_pixel + 4,
                    f"{y}"
                )

                painter.setPen(
                    grid_pen
                )

        # ----------------------------------------------------
        # Main axes
        # ----------------------------------------------------

        axis_pen = QPen(
            QColor(120, 120, 120),
            2
        )

        painter.setPen(
            axis_pen
        )

        painter.drawLine(
            self.width() // 2,
            0,
            self.width() // 2,
            self.height()
        )

        painter.drawLine(
            0,
            self.height(),
            self.width(),
            self.height()
        )

        # ----------------------------------------------------
        # Axis labels
        # ----------------------------------------------------

        painter.setPen(
            QPen(Qt.white)
        )

        painter.setFont(
            QFont(
                "Arial",
                12,
                QFont.Bold
            )
        )

        painter.drawText(
            self.width() // 2 + 10,
            20,
            "Range (m)"
        )

        painter.drawText(
            self.width() - 90,
            self.height() - 35,
            "Azimuth (m)"
        )

        painter.end()

    # ========================================================
    # COORDINATE CONVERSION
    # ========================================================

    def normalize_x(self, x):

        return int(
            (
                (x - self.min_x)
                /
                (self.max_x - self.min_x)
            )
            *
            self.width()
        )

    def normalize_y(self, y):

        return int(
            self.height()
            -
            (y / self.max_y)
            *
            self.height()
        )

    # ========================================================
    # GET WIDGET FROM POOL
    # ========================================================

    def _get_widget_from_pool(
        self,
        pool,
        pool_counter,
        max_count
    ):

        if (
            pool_counter < max_count
            and
            pool_counter < len(pool)
        ):

            return pool[
                pool_counter
            ]

        return None

    # ========================================================
    # SETUP POINT
    # ========================================================

    def _setup_point_widget(
        self,
        widget,
        x_pixel,
        y_pixel,
        point_type="detected"
    ):

        if point_type == "detected":

            style = (
                "background-color: #9400D3;"
                "border-radius: 3px;"
            )

            size = (8, 8)

        elif point_type == "tracked":

            style = (
                "background-color: blue;"
                "border: 2px solid blue;"
                "border-radius: 4px;"
            )

            size = (12, 12)

        elif point_type == "static_interest":

            style = (
                "background-color: red;"
                "border: 3px solid red;"
                "border-radius: 6px;"
            )

            size = (16, 16)

        elif point_type == "dynamic_interest":

            style = (
                "background-color: red;"
                "border: 3px solid red;"
                "border-radius: 6px;"
            )

            size = (16, 16)

        elif point_type == "cluster":

            style = (
                "background-color: yellow;"
                "border-radius: 3px;"
            )

            size = (8, 8)

        else:

            style = (
                "background-color: white;"
                "border-radius: 2px;"
            )

            size = (6, 6)

        widget.setStyleSheet(
            style
        )

        widget.resize(
            *size
        )

        widget.move(
            x_pixel - size[0] // 2,
            y_pixel - size[1] // 2
        )

        widget.show()

    # ========================================================
    # HIDE ALL WIDGETS
    # ========================================================

    def _hide_all_widgets(self):

        for widget in self.point_pool:
            widget.hide()

        for widget in self.track_pool:
            widget.hide()

        for widget in self.cluster_pool:
            widget.hide()

        for widget in self.static_pool:
            widget.hide()

        for widget in self.dynamic_pool:
            widget.hide()

        for widget in self.text_pool:
            widget.hide()

    # ========================================================
    # UPDATE RADAR DATA
    # ========================================================

    def update_data(self, json_data):

        try:

            # ------------------------------------------------
            # Reset counters
            # ------------------------------------------------

            self.active_points = 0
            self.active_tracks = 0
            self.active_static = 0
            self.active_dynamic = 0
            self.active_text = 0
            self.active_clusters = 0

            self._hide_all_widgets()

            # ------------------------------------------------
            # Extract data
            # ------------------------------------------------

            detected_points = []

            tracked_objects = []

            clusters = []

            if (
                'Point_cloud' in json_data
                and
                isinstance(
                    json_data['Point_cloud'],
                    dict
                )
            ):

                detected_points = (
                    json_data[
                        'Point_cloud'
                    ].get(
                        'Detect_Points',
                        []
                    )
                )

            if (
                'Tracked_obj' in json_data
                and
                isinstance(
                    json_data['Tracked_obj'],
                    dict
                )
            ):

                tracked_objects = (
                    json_data[
                        'Tracked_obj'
                    ].get(
                        'Track_Objects',
                        []
                    )
                )

            if (
                'Cluster_obj' in json_data
                and
                isinstance(
                    json_data['Cluster_obj'],
                    dict
                )
            ):

                clusters = (
                    json_data[
                        'Cluster_obj'
                    ].get(
                        'Clusters',
                        []
                    )
                )

            # ------------------------------------------------
            # Detected points
            # ------------------------------------------------

            for i, point in enumerate(
                detected_points[
                    :self.max_points
                ]
            ):

                widget = (
                    self._get_widget_from_pool(
                        self.point_pool,
                        i,
                        self.max_points
                    )
                )

                if widget is None:
                    continue

                try:

                    x = float(
                        point.get(
                            'D_X',
                            0
                        )
                    )

                    y = float(
                        point.get(
                            'D_Y',
                            0
                        )
                    )

                except (
                    ValueError,
                    TypeError
                ):

                    continue

                x_pixel = self.normalize_x(x)

                y_pixel = self.normalize_y(y)

                self._setup_point_widget(
                    widget,
                    x_pixel,
                    y_pixel,
                    "detected"
                )

                self.active_points += 1

            # ------------------------------------------------
            # Tracked objects
            # ------------------------------------------------

            for i, track in enumerate(
                tracked_objects[
                    :self.max_tracks
                ]
            ):

                widget = (
                    self._get_widget_from_pool(
                        self.track_pool,
                        i,
                        self.max_tracks
                    )
                )

                if widget is None:
                    continue

                try:

                    x = float(
                        track.get(
                            'T_X',
                            0
                        )
                    )

                    y = float(
                        track.get(
                            'T_Y',
                            0
                        )
                    )

                except (
                    ValueError,
                    TypeError
                ):

                    continue

                x_pixel = self.normalize_x(x)

                y_pixel = self.normalize_y(y)

                self._setup_point_widget(
                    widget,
                    x_pixel,
                    y_pixel,
                    "tracked"
                )

                self.active_tracks += 1

            # ------------------------------------------------
            # Clusters
            # ------------------------------------------------

            for i, cluster in enumerate(
                clusters[
                    :self.max_clusters
                ]
            ):

                widget = (
                    self._get_widget_from_pool(
                        self.cluster_pool,
                        i,
                        self.max_clusters
                    )
                )

                if widget is None:
                    continue

                try:

                    x = float(
                        cluster.get(
                            'C_X',
                            0
                        )
                    )

                    y = float(
                        cluster.get(
                            'C_Y',
                            0
                        )
                    )

                except (
                    ValueError,
                    TypeError
                ):

                    continue

                x_pixel = self.normalize_x(x)

                y_pixel = self.normalize_y(y)

                self._setup_point_widget(
                    widget,
                    x_pixel,
                    y_pixel,
                    "cluster"
                )

                self.active_clusters += 1

            # ------------------------------------------------
            # Static object
            # ------------------------------------------------

            static_obj = json_data.get(
                'Static_obj'
            )

            if (
                isinstance(
                    static_obj,
                    dict
                )
                and static_obj
            ):

                widget = (
                    self.static_pool[0]
                )

                try:

                    # Keeping your original coordinate
                    # convention:
                    #
                    # x_val -> radar Y
                    # y_val -> radar X

                    y = float(
                        static_obj.get(
                            'x_val',
                            0
                        )
                    )

                    x = float(
                        static_obj.get(
                            'y_val',
                            0
                        )
                    )

                except (
                    ValueError,
                    TypeError
                ):

                    x = 0
                    y = 0

                x_pixel = self.normalize_x(x)

                y_pixel = self.normalize_y(y)

                self._setup_point_widget(
                    widget,
                    x_pixel,
                    y_pixel,
                    "static_interest"
                )

                self.active_static = 1

                if self.active_text < self.max_text:

                    text_widget = (
                        self.text_pool[
                            self.active_text
                        ]
                    )

                    text_widget.setText(
                        f"({x:.1f}, {y:.1f})"
                    )

                    text_widget.setStyleSheet(
                        """
                        color: white;
                        background-color:
                            rgba(0,0,0,180);
                        padding: 3px 6px;
                        border-radius: 4px;
                        font: bold 8px Arial;
                        """
                    )

                    text_widget.adjustSize()

                    text_widget.move(
                        x_pixel + 8,
                        y_pixel - 18
                    )

                    text_widget.show()

                    self.active_text += 1

            # ------------------------------------------------
            # Dynamic object
            # ------------------------------------------------

            dynamic_obj = json_data.get(
                'Dynamic_obj'
            )

            if (
                isinstance(
                    dynamic_obj,
                    dict
                )
                and dynamic_obj
            ):

                widget = (
                    self.dynamic_pool[0]
                )

                try:

                    # Keeping your original coordinate
                    # convention.

                    y = float(
                        dynamic_obj.get(
                            'x_val',
                            0
                        )
                    )

                    x = float(
                        dynamic_obj.get(
                            'y_val',
                            0
                        )
                    )

                except (
                    ValueError,
                    TypeError
                ):

                    x = 0
                    y = 0

                x_pixel = self.normalize_x(x)

                y_pixel = self.normalize_y(y)

                self._setup_point_widget(
                    widget,
                    x_pixel,
                    y_pixel,
                    "dynamic_interest"
                )

                self.active_dynamic = 1

                if self.active_text < self.max_text:

                    text_widget = (
                        self.text_pool[
                            self.active_text
                        ]
                    )

                    text_widget.setText(
                        f"({x:.1f}, {y:.1f})"
                    )

                    text_widget.setStyleSheet(
                        """
                        color: white;
                        background-color:
                            rgba(0,0,0,180);
                        padding: 3px 6px;
                        border-radius: 4px;
                        font: bold 8px Arial;
                        """
                    )

                    text_widget.adjustSize()

                    text_widget.move(
                        x_pixel + 8,
                        y_pixel - 18
                    )

                    text_widget.show()

                    self.active_text += 1

        except Exception as e:

            print(
                f"Error updating radar plot: {e}"
            )

    # ========================================================
    # PAINT EVENT
    # ========================================================

    def paintEvent(self, event):

        if self.background_pixmap:

            painter = QPainter(
                self
            )

            painter.drawPixmap(
                0,
                0,
                self.background_pixmap
            )

    # ========================================================
    # RESIZE EVENT
    # ========================================================

    def resizeEvent(self, event):

        super().resizeEvent(
            event
        )

        self.create_background()

        self.update()


# ============================================================
# DATA TABLE
# ============================================================

class DataTableWidget(QFrame):
    """
    Reusable table widget for radar data.
    """

    def __init__(
        self,
        title,
        headers,
        parent=None
    ):

        super().__init__(
            parent
        )

        self.setFrameStyle(
            QFrame.Box
        )

        self.setLineWidth(
            1
        )

        layout = QVBoxLayout()

        # ----------------------------------------------------
        # Title
        # ----------------------------------------------------

        title_label = QLabel(
            title
        )

        title_label.setAlignment(
            Qt.AlignCenter
        )

        title_label.setFont(
            QFont(
                "Arial",
                10,
                QFont.Bold
            )
        )

        title_label.setStyleSheet(
            """
            color: white;
            background-color: #2e2e2e;
            padding: 5px;
            """
        )

        layout.addWidget(
            title_label
        )

        # ----------------------------------------------------
        # Table
        # ----------------------------------------------------

        self.table = QTableWidget(
            1,
            len(headers)
        )

        self.table.setHorizontalHeaderLabels(
            headers
        )

        self.table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch
        )

        self.table.setMaximumHeight(
            80
        )

        self.table.setStyleSheet(
            """
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
            """
        )

        layout.addWidget(
            self.table
        )

        self.setLayout(
            layout
        )

        # ----------------------------------------------------
        # Pre-create items
        # ----------------------------------------------------

        self.table_items = []

        for col in range(
            len(headers)
        ):

            item = QTableWidgetItem(
                ""
            )

            item.setTextAlignment(
                Qt.AlignCenter
            )

            self.table.setItem(
                0,
                col,
                item
            )

            self.table_items.append(
                item
            )

    # ========================================================
    # UPDATE TABLE
    # ========================================================

    def update_data(
        self,
        data_list
    ):

        for col, value in enumerate(
            data_list
        ):

            if col < len(
                self.table_items
            ):

                self.table_items[
                    col
                ].setText(
                    str(value)
                )


# ============================================================
# MAIN WINDOW
# ============================================================

class RadarVisualizerMainWindow(
    QMainWindow
):

    # ========================================================
    # FORMAT VALUE
    # ========================================================

    def format_value(
        self,
        value,
        decimal_places=1
    ):

        """
        Safely convert a value to a formatted string.
        """

        if value is None:
            return "N/A"

        if isinstance(
            value,
            str
        ):

            value = value.strip()

            if value == "":
                return "N/A"

            if value.upper() == "N/A":
                return "N/A"

        try:

            return f"{float(value):.{decimal_places}f}"

        except (
            ValueError,
            TypeError
        ):

            return str(value)

    # ========================================================
    # CONSTRUCTOR
    # ========================================================

    def __init__(self):

        super().__init__()

        self.setWindowTitle(
            "Real-Time Radar Visualizer"
        )

        self.setGeometry(
            100,
            100,
            1400,
            800
        )

        self.setStyleSheet(
            """
            QMainWindow {
                background-color: #1e1e1e;
                color: white;
            }
            """
        )

        self.setup_ui()

        # ----------------------------------------------------
        # Socket thread
        # ----------------------------------------------------

        self.socket_thread = (
            SocketClientThread()
        )

        self.socket_thread.data_received.connect(
            self.update_with_json
        )

        self.socket_thread.start()

        # ----------------------------------------------------
        # Timestamp timer
        # ----------------------------------------------------

        self.timestamp_timer = QTimer()

        self.timestamp_timer.timeout.connect(
            self.update_timestamp
        )

        self.timestamp_timer.start(
            100
        )

    # ========================================================
    # SETUP UI
    # ========================================================

    def setup_ui(self):

        central_widget = QWidget()

        self.setCentralWidget(
            central_widget
        )

        main_layout = QHBoxLayout()

        central_widget.setLayout(
            main_layout
        )

        # ====================================================
        # RADAR
        # ====================================================

        self.plot_widget = (
            RadarPlotWidget()
        )

        main_layout.addWidget(
            self.plot_widget,
            75
        )

        # ====================================================
        # RIGHT PANEL
        # ====================================================

        right_panel = QWidget()

        right_panel.setMaximumWidth(
            300
        )

        right_layout = QVBoxLayout()

        right_panel.setLayout(
            right_layout
        )

        # ====================================================
        # TABLES
        # ====================================================

        self.static_obj_table = (
            DataTableWidget(
                "Static Object of Interest",
                ["X (m)", "Y (m)"]
            )
        )

        self.dynamic_obj_table = (
            DataTableWidget(
                "Dynamic Object of Interest",
                [
                    "X (m)",
                    "Y (m)",
                    "Vx (m/s)",
                    "Vy (m/s)"
                ]
            )
        )

        self.static_alarm_table = (
            DataTableWidget(
                "Static Alarm Threshold",
                [
                    "L1",
                    "L2",
                    "L3"
                ]
            )
        )

        self.dynamic_alarm_table = (
            DataTableWidget(
                "Dynamic Alarm Threshold",
                [
                    "L1",
                    "L2",
                    "L3"
                ]
            )
        )

        self.alarm_status_table = (
            DataTableWidget(
                "Alarm Status",
                ["Alarms"]
            )
        )

        right_layout.addWidget(
            self.static_obj_table
        )

        right_layout.addWidget(
            self.dynamic_obj_table
        )

        right_layout.addWidget(
            self.static_alarm_table
        )

        right_layout.addWidget(
            self.dynamic_alarm_table
        )

        right_layout.addWidget(
            self.alarm_status_table
        )

        right_layout.addStretch()

        main_layout.addWidget(
            right_panel,
            25
        )

        # ====================================================
        # EGO SPEED LABEL
        # ====================================================

        self.ego_speed_label = QLabel(
            "Ego Speed: N/A km/h",
            self
        )

        self.ego_speed_label.setStyleSheet(
            """
            QLabel {
                color: white;
                background-color:
                    rgba(0, 0, 0, 180);
                padding: 8px 12px;
                font: bold 12px Arial;
            }
            """
        )

        self.ego_speed_label.adjustSize()

        self.ego_speed_label.move(
            10,
            10
        )

        # ====================================================
        # TIMESTAMP LABEL
        # ====================================================

        self.timestamp_label = QLabel(
            "Timestamp: N/A",
            self
        )

        self.timestamp_label.setStyleSheet(
            """
            QLabel {
                color: white;
                background-color:
                    rgba(0, 0, 0, 180);
                padding: 6px 10px;
                font: 10px Arial;
            }
            """
        )

        self.timestamp_label.adjustSize()

        # ====================================================
        # OBJECT INFORMATION LABELS
        # ====================================================

        label_x = 10
        label_y = 400
        label_spacing = 28

        label_style = """
            QLabel {
                color: white;
                background-color:
                    rgba(0, 0, 0, 180);
                padding: 6px 10px;
                font: bold 10px Arial;
            }
        """

        # ----------------------------------------------------
        # Object ID
        # ----------------------------------------------------

        self.nObjectIdentificationLabel = QLabel(
            "ObjectID                : N/A",
            self
        )

        self.nObjectIdentificationLabel.setStyleSheet(
            label_style
        )

        self.nObjectIdentificationLabel.adjustSize()

        self.nObjectIdentificationLabel.move(
            label_x,
            label_y
        )

        # ----------------------------------------------------
        # Distance X
        # ----------------------------------------------------

        self.nDistXLabel = QLabel(
            "distX                   : N/A",
            self
        )

        self.nDistXLabel.setStyleSheet(
            label_style
        )

        self.nDistXLabel.adjustSize()

        self.nDistXLabel.move(
            label_x,
            label_y + label_spacing
        )

        # ----------------------------------------------------
        # Distance Y
        # ----------------------------------------------------

        self.nDistYLabel = QLabel(
            "distY                   : N/A",
            self
        )

        self.nDistYLabel.setStyleSheet(
            label_style
        )

        self.nDistYLabel.adjustSize()

        self.nDistYLabel.move(
            label_x,
            label_y + label_spacing * 2
        )

        # ----------------------------------------------------
        # Relative Velocity X
        # ----------------------------------------------------

        self.nvRelXLabel = QLabel(
            "vRelX                   : N/A",
            self
        )

        self.nvRelXLabel.setStyleSheet(
            label_style
        )

        self.nvRelXLabel.adjustSize()

        self.nvRelXLabel.move(
            label_x,
            label_y + label_spacing * 3
        )

        # ----------------------------------------------------
        # Relative Velocity Y
        # ----------------------------------------------------

        self.nvRelYLabel = QLabel(
            "vRelY                   : N/A",
            self
        )

        self.nvRelYLabel.setStyleSheet(
            label_style
        )

        self.nvRelYLabel.adjustSize()

        self.nvRelYLabel.move(
            label_x,
            label_y + label_spacing * 4
        )

        # ----------------------------------------------------
        # Relative Acceleration X
        # ----------------------------------------------------

        self.naRelXLabel = QLabel(
            "aRelX                   : N/A",
            self
        )

        self.naRelXLabel.setStyleSheet(
            label_style
        )

        self.naRelXLabel.adjustSize()

        self.naRelXLabel.move(
            label_x,
            label_y + label_spacing * 5
        )

        # ----------------------------------------------------
        # Relative Acceleration Y
        # ----------------------------------------------------

        self.naRelYLabel = QLabel(
            "aRelY                   : N/A",
            self
        )

        self.naRelYLabel.setStyleSheet(
            label_style
        )

        self.naRelYLabel.adjustSize()

        self.naRelYLabel.move(
            label_x,
            label_y + label_spacing * 6
        )

        # ----------------------------------------------------
        # Orientation
        # ----------------------------------------------------

        self.nOrientation = QLabel(
            "Orientation             : N/A",
            self
        )

        self.nOrientation.setStyleSheet(
            label_style
        )

        self.nOrientation.adjustSize()

        self.nOrientation.move(
            label_x,
            label_y + label_spacing * 7
        )

        # ----------------------------------------------------
        # Dynamic Property
        # ----------------------------------------------------

        self.nDynamicProperty = QLabel(
            "Dynamic Property        : N/A",
            self
        )

        self.nDynamicProperty.setStyleSheet(
            label_style
        )

        self.nDynamicProperty.adjustSize()

        self.nDynamicProperty.move(
            label_x,
            label_y + label_spacing * 8
        )

        # ----------------------------------------------------
        # Classification
        # ----------------------------------------------------

        self.nClassification = QLabel(
            "Classification           : N/A",
            self
        )

        self.nClassification.setStyleSheet(
            label_style
        )

        self.nClassification.adjustSize()

        self.nClassification.move(
            label_x,
            label_y + label_spacing * 9
        )

    # ========================================================
    # TIMESTAMP
    # ========================================================

    def update_timestamp(self):

        try:

            current_timestamp = time.time()

            dt = datetime.datetime.fromtimestamp(
                current_timestamp
            )

            formatted_time = dt.strftime(
                "%Y-%m-%d %H:%M:%S"
            )

            unix_time_str = (
                f"{current_timestamp:.0f}"
            )

            self.timestamp_label.setText(
                f"Time: {formatted_time} "
                f"(Unix: {unix_time_str})"
            )

            self.timestamp_label.adjustSize()

        except (
            ValueError,
            TypeError,
            OSError
        ):

            self.timestamp_label.setText(
                f"Timestamp: {time.time()}"
            )

            self.timestamp_label.adjustSize()

    # ========================================================
    # MAIN JSON UPDATE
    # ========================================================

    def update_with_json(
        self,
        json_data
    ):

        try:

            if not isinstance(
                json_data,
                dict
            ):

                return

            # ------------------------------------------------
            # Radar
            # ------------------------------------------------

            self.plot_widget.update_data(
                json_data
            )

            # ------------------------------------------------
            # Ego speed
            # ------------------------------------------------

            ego_speed = json_data.get(
                'ego_speed'
            )

            speed_value = None

            if isinstance(
                ego_speed,
                dict
            ):

                speed_value = ego_speed.get(
                    'ego_speed'
                )

            else:

                speed_value = ego_speed

            try:

                speed_value = float(
                    speed_value
                )

                if speed_value >= 0:

                    self.ego_speed_label.setText(
                        f"Ego Speed: "
                        f"{speed_value:.1f} km/h"
                    )

                else:

                    self.ego_speed_label.setText(
                        "Ego Speed: N/A km/h"
                    )

            except (
                ValueError,
                TypeError
            ):

                self.ego_speed_label.setText(
                    "Ego Speed: N/A km/h"
                )

            self.ego_speed_label.adjustSize()

            # ------------------------------------------------
            # Tables
            # ------------------------------------------------

            self._update_tables(
                json_data
            )

            # ------------------------------------------------
            # Individual labels
            # ------------------------------------------------

            self._nUpdateMultipleLabelValues(
                json_data
            )

        except Exception as e:

            print(
                f"Error updating display: {e}"
            )

    # ========================================================
    # INDIVIDUAL OBJECT LABELS
    # ========================================================

    def _nUpdateMultipleLabelValues(
        self,
        json_data
    ):
        """
        Update all individual object-information labels.

        Dynamic_obj has priority over Static_obj.

        Expected fields can include:

            ObjectID
            x_val
            y_val
            Vx_val
            Vy_val
            Ax_val
            Ay_val
            Orientation
            DynamicProperty
            Classification

        Multiple alternative key names are supported.
        """

        # ====================================================
        # HELPER FUNCTIONS
        # ====================================================

        def get_value(
            obj,
            *keys,
            default="N/A"
        ):
            """
            Return first available key.
            """

            if not isinstance(
                obj,
                dict
            ):

                return default

            for key in keys:

                if (
                    key in obj
                    and
                    obj[key] is not None
                ):

                    return obj[key]

            return default

        def format_numeric(
            value,
            decimals=4
        ):
            """
            Convert numeric value into formatted string.
            """

            if value is None:

                return "N/A"

            if isinstance(
                value,
                str
            ):

                value = value.strip()

                if value == "":
                    return "N/A"

                if value.upper() == "N/A":
                    return "N/A"

            try:

                # return (
                #     f"{float(value):."
                #     f"{decimals}f}"
                # )


                return f"{float(value):.{decimals}f}"


            except (
                ValueError,
                TypeError
            ):

                return str(value)

        def format_text(
            value
        ):

            if value is None:

                return "N/A"

            text = str(
                value
            ).strip()

            if text == "":
                return "N/A"

            return text

        # ====================================================
        # SELECT OBJECT
        # ====================================================

        dynamic_obj = json_data.get(
            'Dynamic_obj'
        )

        static_obj = json_data.get(
            'Static_obj'
        )

        if (
            isinstance(
                dynamic_obj,
                dict
            )
            and dynamic_obj
        ):

            obj = dynamic_obj

        elif (
            isinstance(
                static_obj,
                dict
            )
            and static_obj
        ):

            obj = static_obj

        else:

            # ------------------------------------------------
            # No object available
            # ------------------------------------------------

            self.nObjectIdentificationLabel.setText(
                "ObjectID                : N/A"
            )

            self.nDistXLabel.setText(
                "distX                   : N/A"
            )

            self.nDistYLabel.setText(
                "distY                   : N/A"
            )

            self.nvRelXLabel.setText(
                "vRelX                   : N/A"
            )

            self.nvRelYLabel.setText(
                "vRelY                   : N/A"
            )

            self.naRelXLabel.setText(
                "aRelX                   : N/A"
            )

            self.naRelYLabel.setText(
                "aRelY                   : N/A"
            )

            self.nOrientation.setText(
                "Orientation             : N/A"
            )

            self.nDynamicProperty.setText(
                "Dynamic Property        : N/A"
            )

            self.nClassification.setText(
                "Classification           : N/A"
            )

            return

        # ====================================================
        # OBJECT ID
        # ====================================================

        object_id = get_value(
            obj,
            "ObjectID",
            "Object_Id",
            "object_id",
            "ObjID",
            "Obj_Id",
            "ID",
            "id"
        )

        # ====================================================
        # POSITION
        # ====================================================

        dist_x = get_value(
            obj,
            "x_val",
            "X_val",
            "distX",
            "DistX",
            "x",
            "X"
        )

        dist_y = get_value(
            obj,
            "y_val",
            "Y_val",
            "distY",
            "DistY",
            "y",
            "Y"
        )

        # ====================================================
        # VELOCITY
        # ====================================================

        v_rel_x = get_value(
            obj,
            "Vx_val",
            "vx_val",
            "vRelX",
            "VRelX",
            "Vx",
            "vx"
        )

        v_rel_y = get_value(
            obj,
            "Vy_val",
            "vy_val",
            "vRelY",
            "VRelY",
            "Vy",
            "vy"
        )

        # ====================================================
        # ACCELERATION
        # ====================================================

        a_rel_x = get_value(
            obj,
            "Ax_val",
            "ax_val",
            "aRelX",
            "ARelX",
            "Ax",
            "ax"
        )

        a_rel_y = get_value(
            obj,
            "Ay_val",
            "ay_val",
            "aRelY",
            "ARelY",
            "Ay",
            "ay"
        )

        # ====================================================
        # ORIENTATION
        # ====================================================

        orientation = get_value(
            obj,
            "Orientation",
            "orientation",
            "Orientation_val",
            "orientation_val",
            "Yaw",
            "yaw",
            "Heading",
            "heading"
        )

        # ====================================================
        # DYNAMIC PROPERTY
        # ====================================================

        dynamic_property = get_value(
            obj,
            "DynamicProperty",
            "dynamic_property",
            "Dynamic_Property",
            "dynamicProperty",
            "DynProperty",
            "dynamic"
        )

        # ====================================================
        # CLASSIFICATION
        # ====================================================

        classification = get_value(
            obj,
            "Classification",
            "classification",
            "Class",
            "class",
            "Class_Type",
            "Object_Class",
            "object_class"
        )

        # ====================================================
        # FORMAT VALUES
        # ====================================================

        object_id_text = format_text(
            object_id
        )

        dist_x_text = format_numeric(
            dist_x,
            4
        )

        dist_y_text = format_numeric(
            dist_y,
            4
        )

        v_rel_x_text = format_numeric(
            v_rel_x,
            4
        )

        v_rel_y_text = format_numeric(
            v_rel_y,
            4
        )

        a_rel_x_text = format_numeric(
            a_rel_x,
            4
        )

        a_rel_y_text = format_numeric(
            a_rel_y,
            4
        )

        orientation_text = format_numeric(
            orientation,
            2
        )

        dynamic_property_text = format_text(
            dynamic_property
        )

        classification_text = format_text(
            classification
        )

        # ====================================================
        # UPDATE LABELS
        # ====================================================

        self.nObjectIdentificationLabel.setText(
            f"ObjectID                : "
            f"{object_id_text}"
        )

        self.nDistXLabel.setText(
            f"distX                   : "
            f"{dist_x_text} m"
        )

        self.nDistYLabel.setText(
            f"distY                   : "
            f"{dist_y_text} m"
        )

        self.nvRelXLabel.setText(
            f"vRelX                   : "
            f"{v_rel_x_text} m/s"
        )

        self.nvRelYLabel.setText(
            f"vRelY                   : "
            f"{v_rel_y_text} m/s"
        )

        self.naRelXLabel.setText(
            f"aRelX                   : "
            f"{a_rel_x_text} m/s²"
        )

        self.naRelYLabel.setText(
            f"aRelY                   : "
            f"{a_rel_y_text} m/s²"
        )

        self.nOrientation.setText(
            f"Orientation             : "
            f"{orientation_text}°"
        )

        self.nDynamicProperty.setText(
            f"Dynamic Property        : "
            f"{dynamic_property_text}"
        )

        self.nClassification.setText(
            f"Classification           : "
            f"{classification_text}"
        )

        # ====================================================
        # ADJUST LABEL SIZES
        # ====================================================

        self.nObjectIdentificationLabel.adjustSize()

        self.nDistXLabel.adjustSize()

        self.nDistYLabel.adjustSize()

        self.nvRelXLabel.adjustSize()

        self.nvRelYLabel.adjustSize()

        self.naRelXLabel.adjustSize()

        self.naRelYLabel.adjustSize()

        self.nOrientation.adjustSize()

        self.nDynamicProperty.adjustSize()

        self.nClassification.adjustSize()

    # ========================================================
    # UPDATE TABLES
    # ========================================================

    def _update_tables(
        self,
        json_data
    ):

        # ====================================================
        # STATIC OBJECT
        # ====================================================

        static_obj = json_data.get(
            'Static_obj'
        )

        if (
            isinstance(
                static_obj,
                dict
            )
            and static_obj
        ):

            static_data = [

                self.format_value(
                    static_obj.get(
                        'x_val',
                        'N/A'
                    )
                ),

                self.format_value(
                    static_obj.get(
                        'y_val',
                        'N/A'
                    )
                )
            ]

            self.static_obj_table.update_data(
                static_data
            )

        # ====================================================
        # DYNAMIC OBJECT
        # ====================================================

        dynamic_obj = json_data.get(
            'Dynamic_obj'
        )

        if (
            isinstance(
                dynamic_obj,
                dict
            )
            and dynamic_obj
        ):

            dynamic_data = [

                self.format_value(
                    dynamic_obj.get(
                        'x_val',
                        'N/A'
                    )
                ),

                self.format_value(
                    dynamic_obj.get(
                        'y_val',
                        'N/A'
                    )
                ),

                self.format_value(
                    dynamic_obj.get(
                        'Vx_val',
                        'N/A'
                    )
                ),

                self.format_value(
                    dynamic_obj.get(
                        'Vy_val',
                        'N/A'
                    )
                )
            ]

            self.dynamic_obj_table.update_data(
                dynamic_data
            )

        # ====================================================
        # ALARM THRESHOLDS
        # ====================================================

        alarm_thresh = json_data.get(
            'Alarm_thresh'
        )

        if (
            isinstance(
                alarm_thresh,
                dict
            )
            and alarm_thresh
        ):

            static_alarm_data = [

                self.format_value(
                    alarm_thresh.get(
                        'Level_1_Sta',
                        'N/A'
                    )
                ),

                self.format_value(
                    alarm_thresh.get(
                        'Level_2_Sta',
                        'N/A'
                    )
                ),

                self.format_value(
                    alarm_thresh.get(
                        'Level_3_Sta',
                        'N/A'
                    )
                )
            ]

            self.static_alarm_table.update_data(
                static_alarm_data
            )

            dynamic_alarm_data = [

                self.format_value(
                    alarm_thresh.get(
                        'Level_1_Mov',
                        'N/A'
                    )
                ),

                self.format_value(
                    alarm_thresh.get(
                        'Level_2_Mov',
                        'N/A'
                    )
                ),

                self.format_value(
                    alarm_thresh.get(
                        'Level_3_Mov',
                        'N/A'
                    )
                )
            ]

            self.dynamic_alarm_table.update_data(
                dynamic_alarm_data
            )

        # ====================================================
        # ALARM STATUS
        # ====================================================

        global count

        alarm_status = json_data.get(
            'Alarm_status'
        )

        if (
            isinstance(
                alarm_status,
                dict
            )
            and alarm_status
        ):

            alarm_type = alarm_status.get(
                'Alarm_type',
                0
            )

            alarm_text = (
                f"Alarm {alarm_type}"
            )

            self.alarm_status_table.update_data(
                [alarm_text]
            )

            count = 0

        else:

            if count > 3:

                self.alarm_status_table.update_data(
                    ["No Alarm"]
                )

                count = 0

            count += 1

    # ========================================================
    # WINDOW RESIZE
    # ========================================================

    def resizeEvent(
        self,
        event
    ):

        super().resizeEvent(
            event
        )

        self.timestamp_label.move(
            10,
            self.height()
            -
            self.timestamp_label.height()
            -
            30
        )

    # ========================================================
    # CLOSE EVENT
    # ========================================================

    def closeEvent(
        self,
        event
    ):

        if hasattr(
            self,
            'socket_thread'
        ):

            self.socket_thread.stop()

        if hasattr(
            self,
            'timestamp_timer'
        ):

            self.timestamp_timer.stop()

        event.accept()


# ============================================================
# DATA INPUT INTERFACE
# ============================================================

class RadarDataInterface:
    """
    Interface for receiving JSON data from external sources.
    """

    def __init__(
        self,
        visualizer
    ):

        self.visualizer = visualizer

    def process_frame_data(
        self,
        json_data
    ):

        if isinstance(
            json_data,
            str
        ):

            try:

                json_data = json.loads(
                    json_data
                )

            except json.JSONDecodeError as e:

                print(
                    f"Invalid JSON: {e}"
                )

                return

        if isinstance(
            json_data,
            dict
        ):

            self.visualizer.update_with_json(
                json_data
            )


# ============================================================
# MAIN
# ============================================================

def main():

    app = QApplication(
        sys.argv
    )

    app.setStyle(
        'Fusion'
    )

    main_window = (
        RadarVisualizerMainWindow()
    )

    main_window.show()

    # --------------------------------------------------------
    # External data interface
    # --------------------------------------------------------

    data_interface = (
        RadarDataInterface(
            main_window
        )
    )

    app.radar_interface = (
        data_interface
    )

    sys.exit(
        app.exec_()
    )


# ============================================================
# PROGRAM ENTRY
# ============================================================

if __name__ == "__main__":

    main()
