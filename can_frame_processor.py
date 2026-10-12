#!/usr/bin/env python3

"""
CAN Frame Processor with High-Performance Threading
This code integrates CAN bus message collection with the high-performance frame processing
architecture. It reads CAN messages, identifies frame boundaries based on specific header
and data patterns, and processes complete frames in a separate thread.

Updated from original CAN_Logging.py to implement frame-based processing.
"""

import can
import csv
import threading
import queue
import time
from datetime import datetime
from typing import List, Dict, Any, Optional
import socket
import json
import argparse
import os

import can_source
from object_tracker import ObjectTracker

import sys
if sys.platform.startswith("win") and hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
# CAN Configuration Constants
# TODO: Update these values for your specific system
#OR_MRR_DATA_HEADER = 0x123 # Replace with your actual CAN ID for frame header
OR_MRR_DATA_HEADER = 0xC1
OR_MRR_POINT_CLOUD = 0xD1
OR_MRR_CLUSTERS = 0xD2
OR_MRR_TRACKERS = 0xD3
OR_MRR_PADDING = 0xE1
# DBC V6 exposes three moving and three stationary objects, ranked by
# proximity: _1 is the closest. These are slots, not identities - an object
# moves between slots as the ranking changes.
OR_FCWS_MOVING_OBJ = 0x510
OR_FCWS_MOVING_OBJ_2 = 0x511
OR_FCWS_MOVING_OBJ_3 = 0x512
OR_FCWS_STATIC_OBJ = 0x590
OR_FCWS_STATIC_OBJ_2 = 0x591
OR_FCWS_STATIC_OBJ_3 = 0x592
OR_TIME_MSG = 0x6F0        # time to collision, PET, relevant object index
VEH_YAW = 0x3F7            # ego yaw rate
# Object counts reported by the radar (DBC: FCWS_Dynamic/Stationary_Object_Count).
# These are how many objects the radar says it sees, which is what exposes an
# object split - 0x510/0x590 only ever carry the single closest object.
OR_FCWS_DYNAMIC_COUNT = 0x501
OR_FCWS_STATIC_COUNT = 0x581
OR_ALARM_STATUS = 0xA1
OR_FCWS_THRESH = 0x6F1
VEH_SPEED = 0x3F5
OR_ECU_ALARM = 0x1
HIL_100 =0x100
HIL_101 =0x101
HIL_102 =0x102
HIL_103 =0x103

can_ids_to_filter = [
    OR_MRR_DATA_HEADER, OR_MRR_POINT_CLOUD, OR_MRR_TRACKERS, OR_MRR_CLUSTERS,
    OR_FCWS_MOVING_OBJ, OR_FCWS_MOVING_OBJ_2, OR_FCWS_MOVING_OBJ_3,
    OR_FCWS_STATIC_OBJ, OR_FCWS_STATIC_OBJ_2, OR_FCWS_STATIC_OBJ_3,
    OR_ALARM_STATUS, OR_FCWS_THRESH, OR_TIME_MSG, OR_ECU_ALARM,
    VEH_SPEED, VEH_YAW,
    HIL_100, HIL_101, HIL_102, HIL_103,
    OR_FCWS_DYNAMIC_COUNT, OR_FCWS_STATIC_COUNT,
]

FRAME_START_DATA = "02 01 04 03 06 05 08 07"  # Frame delimiter data pattern

# Keepalive sent to the visualizer; a real message so it cannot corrupt a frame
KEEPALIVE_MESSAGE = b'{"_keepalive":1}\n'

# Per-frame tracing is expensive at radar frame rates; opt in with ORDLAV_DEBUG=1
DEBUG = os.environ.get("ORDLAV_DEBUG", "").lower() in ("1", "true", "yes")

class CANFrameProcessor:
    """
    High-performance CAN frame processor that collects CAN messages into frames
    based on specific header and data patterns, then processes complete frames
    in a separate thread for optimal performance.
    """

    def __init__(self, channel='0', bitrate=500000, csv_filename: str = None,
                 source: str = None, replay_speed: float = 1.0,
                 replay_loop: bool = False, raw_log: bool = True,
                 wait_for_visualizer: float = 0.0):
        # CAN bus configuration
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None

        # Message source: hardware, virtual bus, or a replayed CSV log
        self.source = source or can_source.default_source()
        self.replay_speed = replay_speed
        self.replay_loop = replay_loop
        self.raw_log = raw_log
        self.wait_for_visualizer = wait_for_visualizer

        # Set once a finite source (a replayed log) has been fully consumed
        self.source_exhausted = threading.Event()

        # The radar sends no object ID, so identities are assigned here by
        # associating each frame's tracks with the previous frame's.
        self.tracker = ObjectTracker()
        # The CAN object slots are ranked by proximity, so slot 1 becomes
        # slot 2 the moment something closer appears. They are positions in a
        # list, not identities, and need associating just like the TLV tracks.
        self.object_trackers = {
            'Dynamic_objects': ObjectTracker(),
            'Static_objects': ObjectTracker(),
        }
        self._frame_counter = 0

        # Frame processing
        self.frame_queue = queue.Queue(maxsize=1000)
        self.current_frame = []  # Buffer for current frame being collected
        self.frame_active = False  # Whether we're currently collecting a frame

        # Thread control
        self.running = threading.Event()
        self.running.set()

        # CSV logging for raw CAN data
        if csv_filename is None:
            timestamp = datetime.now()
            self.csv_filename = timestamp.strftime('%H_%M_%m_%d_%Y.csv')
        else:
            self.csv_filename = csv_filename

        # CSV logging for parsed data (NEW)
        timestamp = datetime.now()
        stamp = timestamp.strftime('%H_%M_%m_%d_%Y.csv')
        self.parsed_csv_filename = "parsed_" + stamp
        # Long-format companions: one row per frame, and one row per object per
        # frame. The wide parsed CSV keeps its shape so existing consumers are
        # unaffected, but it can only hold one object per category and stores
        # point/cluster/track lists as unparseable repr() strings.
        self.frames_csv_filename = "frames_" + stamp
        self.objects_csv_filename = "objects_" + stamp
        self.parsed_data_queue = queue.Queue(maxsize=1000)

        # Initialize parsed CSV with headers
        self._initialize_parsed_csv()
        self._initialize_long_format_csv()

        # Open log handles once: reopening per message costs more than the
        # rest of the collection loop at bench message rates.
        self._raw_csv_handle = None
        self._raw_csv_writer = None
        self._parsed_csv_handle = None
        self._parsed_csv_writer = None
        self._frames_csv_handle = None
        self._frames_csv_writer = None
        self._objects_csv_handle = None
        self._objects_csv_writer = None
        self._csv_lock = threading.Lock()

        # Performance statistics
        self.frames_collected = 0
        self.frames_processed = 0
        self.messages_received = 0
        self.parsed_logs_written = 0
        self.start_time = time.perf_counter()
        self.stats_lock = threading.Lock()

        # Socket server for visualizer communication
        self.server_socket = None
        self.client_connection = None
        self.socket_running = False
        # The keepalive and the frame sender write to the same socket from
        # different threads; serialize them so messages cannot interleave.
        self.send_lock = threading.Lock()

        # print(f"CAN Frame Processor initialized")
        # print(f"Channel: {self.channel}, Bitrate: {self.bitrate}")
        # print(f"Raw CSV filename: {self.csv_filename}")
        # print(f"Parsed CSV filename: {self.parsed_csv_filename}")
        # print(f"Frame header ID: 0x{OR_MRR_DATA_HEADER:X}")
        # print(f"Frame start data: {FRAME_START_DATA}")

    def _initialize_parsed_csv(self):
        """Initialize the parsed CSV file with headers"""
        try:
            headers = [
                'timestamp', 'frame_num',
                # Dynamic Object
                'dynamic_x', 'dynamic_y', 'dynamic_vx', 'dynamic_vy',
                # Static Object  
                'static_x', 'static_y', 'static_vx', 'static_vy',
                #HIL Target1
                'HIL1_x','HIL1_y','HIL1_vx','HIL1_vy',
                #HIL Target2
                'HIL2_x','HIL2_y','HIL2_vx','HIL2_vy',
                # Ego Speed
                'ego_speed',
                # Alarm Thresholds
                'alarm_level_1_sta', 'alarm_level_2_sta', 'alarm_level_3_sta',
                'alarm_level_1_mov', 'alarm_level_2_mov', 'alarm_level_3_mov',
                # Alarm Status
                'alarm_type',
                # Point Cloud (counts)
                'detected_points',
                # Tracked Objects (counts)
                'tracked_objects',
                # Clustered Objects
                'cluster_objects'
            ]

            with open(self.parsed_csv_filename, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(headers)

          #  print(f"✅ Initialized parsed CSV: {self.parsed_csv_filename}")

        except Exception as e:
            print(f"❌ Error initializing parsed CSV: {e}")

    # One row per frame.
    FRAMES_COLUMNS = [
        'frame_num', 'timestamp', 'ego_speed_mps', 'yaw_rate', 'alarm_type',
        'ttc', 'pet', 'relevant_object',
        'thresh_l1_sta', 'thresh_l2_sta', 'thresh_l3_sta',
        'thresh_l1_mov', 'thresh_l2_mov', 'thresh_l3_mov',
        'n_dynamic_reported', 'n_static_reported', 'radar_cycle',
        'n_points', 'n_clusters', 'n_tracks',
    ]

    # One row per object per frame. Coordinates are normalised to the vehicle
    # convention used throughout the project: distX longitudinal (ahead
    # positive), distY lateral (right positive). The radar's TLV stream uses
    # the opposite axis naming, so it is converted here rather than leaving
    # every consumer to remember which source means what.
    # assoc_id is assigned by object_tracker, not reported by the radar.
    OBJECTS_COLUMNS = [
        'frame_num', 'timestamp', 'source', 'obj_index', 'assoc_id',
        'distX', 'distY', 'vrelX', 'vrelY',
        'extentX', 'extentY', 'doppler', 'peak',
        # Ground-truth attributes; the radar itself reports none of these
        'target_type', 'trajectory', 'rcs_dbsm',
    ]

    def _initialize_long_format_csv(self):
        """Create the long-format CSVs with their headers."""
        try:
            for filename, columns in ((self.frames_csv_filename, self.FRAMES_COLUMNS),
                                      (self.objects_csv_filename, self.OBJECTS_COLUMNS)):
                with open(filename, 'w', newline='') as csvfile:
                    csv.writer(csvfile).writerow(columns)
        except Exception as e:
            print(f"❌ Error initializing long-format CSV: {e}")

    @staticmethod
    def _as_float(value):
        """Values arrive as formatted strings; blank out anything unusable."""
        try:
            return float(value)
        except (TypeError, ValueError):
            return ''

    def _long_format_rows(self, frame_num, timestamp, data):
        """Split one frame's result dict into a frame row and object rows."""
        points = data.get('Point_cloud', {}).get('Detect_Points', []) or []
        clusters = data.get('Cluster_obj', {}).get('Clusters', []) or []
        tracks = data.get('Tracked_obj', {}).get('Track_Objects', []) or []
        counts = data.get('Object_counts', {}) or {}
        thresholds = data.get('Alarm_thresh', {}) or {}

        timing = data.get('Time_msg', {}) or {}
        vehicle = data.get('Vehicle', {}) or {}

        frame_row = [
            frame_num, timestamp,
            self._as_float(data.get('ego_speed', {}).get('ego_speed')),
            vehicle.get('yaw_rate', ''),
            data.get('Alarm_status', {}).get('Alarm_type', ''),
            timing.get('ttc', ''), timing.get('pet', ''),
            timing.get('relevant_object', ''),
            thresholds.get('Level_1_Sta', ''), thresholds.get('Level_2_Sta', ''),
            thresholds.get('Level_3_Sta', ''), thresholds.get('Level_1_Mov', ''),
            thresholds.get('Level_2_Mov', ''), thresholds.get('Level_3_Mov', ''),
            counts.get('dynamic', ''), counts.get('static', ''),
            counts.get('dynamic_cycle', ''),
            len(points), len(clusters), len(tracks),
        ]

        rows = []

        def add(source, index, distX, distY, vrelX='', vrelY='',
                extentX='', extentY='', doppler='', peak='', assoc_id='',
                target_type='', trajectory='', rcs=''):
            rows.append([frame_num, timestamp, source, index, assoc_id,
                         distX, distY, vrelX, vrelY,
                         extentX, extentY, doppler, peak,
                         target_type, trajectory, rcs])

        # CAN objects already use distX/distY naming. Each slot is logged
        # separately; obj_index is the slot, ranked by proximity.
        for source, key in (('dynamic', 'Dynamic_objects'),
                            ('static', 'Static_objects')):
            for slot, payload in sorted((data.get(key) or {}).items()):
                add(source, slot,
                    self._as_float(payload.get('x_val')),
                    self._as_float(payload.get('y_val')),
                    self._as_float(payload.get('Vx_val')),
                    self._as_float(payload.get('Vy_val')),
                    assoc_id=payload.get('assoc_id', ''))

        for target, payload in sorted((data.get('HIL_Targets') or {}).items()):
            add(f'hil{target}', target,
                self._as_float(payload.get('x_val')),
                self._as_float(payload.get('y_val')),
                self._as_float(payload.get('Vx_val')),
                self._as_float(payload.get('Vy_val')),
                target_type=payload.get('type', ''),
                trajectory=payload.get('trajectory', ''),
                rcs=payload.get('rcs_dbsm', ''))

        # TLV sources: X is lateral and Y is range, so the axes swap here
        for index, track in enumerate(tracks):
            add('track', index,
                self._as_float(track.get('T_Y')), self._as_float(track.get('T_X')),
                self._as_float(track.get('T_Vy')), self._as_float(track.get('T_Vx')),
                assoc_id=track.get('T_ID', ''))

        for index, cluster in enumerate(clusters):
            add('cluster', index,
                self._as_float(cluster.get('C_Y')), self._as_float(cluster.get('C_X')),
                extentX=self._as_float(cluster.get('C_SY')),
                extentY=self._as_float(cluster.get('C_SX')))

        for index, point in enumerate(points):
            add('point', index,
                self._as_float(point.get('D_Y')), self._as_float(point.get('D_X')),
                doppler=self._as_float(point.get('Doppler')),
                peak=self._as_float(point.get('Peak')))

        return frame_row, rows

    def setup_socket_server(self):
        """Setup socket server for visualizer communication"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(("localhost", 5001))
            self.server_socket.listen(1)
            self.socket_running = True
            print("🌐 Socket server started on localhost:5001")
            print("📡 Waiting for visualizer connection...")

            # Start socket thread
            self.socket_thread = threading.Thread(
                target=self.handle_socket_connections,
                name="SocketServerThread",
                daemon=True
            )
            self.socket_thread.start()
            return True
        except Exception as e:
            print(f"❌ Failed to setup socket server: {e}")
            return False

    def handle_socket_connections(self):
        """Handle socket connections from visualizer"""
        while self.socket_running:
            try:
                self.client_connection, addr = self.server_socket.accept()
                print(f"✅ Visualizer connected from {addr}")

                # Keep connection alive. The keepalive has to be a proper
                # newline-delimited message: raw bytes could land in the
                # middle of a frame's JSON and corrupt it.
                while self.socket_running:
                    try:
                        with self.send_lock:
                            self.client_connection.sendall(KEEPALIVE_MESSAGE)
                        time.sleep(1)
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        print("📡 Visualizer disconnected, waiting for new connection...")
                        break
            except Exception as e:
                if self.socket_running:
                    print(f"❌ Socket connection error: {e}")
                    time.sleep(1)

    def send_to_visualizer(self, data):
        """Send JSON data to visualizer"""
        if self.client_connection:
            try:
                json_message = json.dumps(data) + "\n"
                with self.send_lock:
                    self.client_connection.sendall(json_message.encode('utf-8'))
            except (BrokenPipeError, ConnectionResetError):
                print("📡 Connection lost with visualizer")
                self.client_connection = None
            except Exception as e:
                print(f"❌ Error sending data to visualizer: {e}")

    def log_parsed_data_async(self, result_data, frame_timestamp=None):
        """Add parsed data to async logging queue"""
        try:
            log_entry = {
                'timestamp': frame_timestamp or time.time(),
                'data': result_data
            }
            self.parsed_data_queue.put_nowait(log_entry)
        except queue.Full:
            print("⚠️ Parsed data queue full - dropping log entry")

    def parsed_csv_writer_thread(self):
        """Asynchronous CSV writer thread for parsed data"""
        #print("📝 Starting parsed CSV writer thread...")
        frame_counter = 0

        while self.running.is_set():
            try:
                # Get log entry with timeout
                log_entry = self.parsed_data_queue.get(timeout=0.1)
                frame_counter += 1

                timestamp = log_entry['timestamp']
                data = log_entry['data']

                # Extract data from result dictionary
                row_data = [
                    timestamp,
                    frame_counter,
                    # Dynamic Object
                    data.get('Dynamic_obj', {}).get('x_val', ''),
                    data.get('Dynamic_obj', {}).get('y_val', ''),
                    data.get('Dynamic_obj', {}).get('Vx_val', ''),
                    data.get('Dynamic_obj', {}).get('Vy_val', ''),
                    # Static Object
                    data.get('Static_obj', {}).get('x_val', ''),
                    data.get('Static_obj', {}).get('y_val', ''),
                    data.get('Static_obj', {}).get('Vx_val', ''),
                    data.get('Static_obj', {}).get('Vy_val', ''),

                    data.get('HIL_Target1', {}).get('x_val', ''),
                    data.get('HIL_Target1', {}).get('y_val', ''),
                    data.get('HIL_Target1', {}).get('Vx_val', ''),
                    data.get('HIL_Target1', {}).get('Vy_val', ''),

                    data.get('HIL_Target2', {}).get('x_val', ''),
                    data.get('HIL_Target2', {}).get('y_val', ''),
                    data.get('HIL_Target2', {}).get('Vx_val', ''),
                    data.get('HIL_Target2', {}).get('Vy_val', ''),

                    # Ego Speed
                    data.get('ego_speed', {}).get('ego_speed', ''),
                    # Alarm Thresholds
                    data.get('Alarm_thresh', {}).get('Level_1_Sta', ''),
                    data.get('Alarm_thresh', {}).get('Level_2_Sta', ''),
                    data.get('Alarm_thresh', {}).get('Level_3_Sta', ''),
                    data.get('Alarm_thresh', {}).get('Level_1_Mov', ''),
                    data.get('Alarm_thresh', {}).get('Level_2_Mov', ''),
                    data.get('Alarm_thresh', {}).get('Level_3_Mov', ''),
                    # Alarm Status
                    data.get('Alarm_status', {}).get('Alarm_type', ''),
                    # Point Cloud Count
                    data.get('Point_cloud', {}).get('Detect_Points', []),
                    # Tracked Objects Count
                    data.get('Tracked_obj', {}).get('Track_Objects', []),
                    # Clustered Objects (list of clusters for this frame)
                    data.get('Cluster_obj', {}).get('Clusters', [])
                ]

                # Write to CSV
                if self._parsed_csv_writer is None:
                    self._parsed_csv_handle = open(self.parsed_csv_filename, 'a', newline='')
                    self._parsed_csv_writer = csv.writer(self._parsed_csv_handle)
                self._parsed_csv_writer.writerow(row_data)

                # Long-format companions
                if self._frames_csv_writer is None:
                    self._frames_csv_handle = open(self.frames_csv_filename, 'a', newline='')
                    self._frames_csv_writer = csv.writer(self._frames_csv_handle)
                    self._objects_csv_handle = open(self.objects_csv_filename, 'a', newline='')
                    self._objects_csv_writer = csv.writer(self._objects_csv_handle)

                frame_row, object_rows = self._long_format_rows(
                    frame_counter, timestamp, data)
                self._frames_csv_writer.writerow(frame_row)
                if object_rows:
                    self._objects_csv_writer.writerows(object_rows)

                with self.stats_lock:
                    self.parsed_logs_written += 1

            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ Parsed CSV writer error: {e}")

        #print(f"📝 Parsed CSV writer thread stopped. Wrote {self.parsed_logs_written} entries.")

    def setup_can_bus(self) -> bool:
        """Initialize CAN bus connection"""
        try:
            filters = [{"can_id": can_id, "can_mask": 0xFFFFFFFF, "extended": False} for can_id in can_ids_to_filter]
            self.bus = can_source.open_source(
                self.source,
                channel=self.channel,
                bitrate=self.bitrate,
                can_filters=filters,
                speed=self.replay_speed,
                loop=self.replay_loop,
            )
            print(f"✅ CAN source connected: {getattr(self.bus, 'channel_info', self.source)}")
            return True
        except Exception as e:
            print(f"❌ Failed to initialize CAN source '{self.source}': {e}")
            return False

    def format_can_data(self, message_data: bytes) -> str:
        """Format CAN message data as space-separated hex string"""
        return ' '.join(format(byte, '02X') for byte in message_data)

    def is_frame_start_message(self, arbitration_id: int, data: bytes) -> bool:
        """
        Check if this CAN message marks the start/end of a frame
        Args:
            arbitration_id: CAN message ID
            data: CAN message data bytes
        Returns:
            True if this is a frame delimiter message
        """
        # Check if arbitration ID matches frame header
        if arbitration_id != OR_MRR_DATA_HEADER:
            return False

        # Check if data matches frame start pattern
        formatted_data = self.format_can_data(data)
        return formatted_data == FRAME_START_DATA

    def collect_can_messages(self):
        """
        Main CAN message collection thread.
        This implements the logic from your requirements:
        1. Check if arbitration_id equals OR_MRR_DATA_HEADER
        1.a. If yes, check if data equals "02 01 04 03 06 05 08 07"
        1.a.i. If yes, start/end frame collection
        1.a.ii. If no, continue to next message
        1.b. If no, continue to next message
        2. Send complete frames to parse_frame_thread
        """
        if not self.bus:
            print("❌ CAN bus not initialized")
            return

       # print("🚀 Starting CAN message collection...")

        try:
            while self.running.is_set():
                # Receive CAN message with timeout for responsive shutdown
                message = self.bus.recv(timeout=0.1)
                if message is None:
                    # A replayed log is finite: signal the end instead of
                    # polling an empty source forever.
                    if getattr(self.bus, 'exhausted', False):
                        if self.frame_active and self.current_frame:
                            try:
                                self.frame_queue.put_nowait(self.current_frame.copy())
                                with self.stats_lock:
                                    self.frames_collected += 1
                            except queue.Full:
                                pass
                        self.source_exhausted.set()
                        break
                    continue

                # Update statistics
                with self.stats_lock:
                    self.messages_received += 1

                # Extract message fields
                timestamp = message.timestamp
                arbitration_id = message.arbitration_id
                dlc = message.dlc
                data = message.data

                # Log all messages to CSV (as in original code)
                if self.raw_log:
                    self.log_to_csv(timestamp, arbitration_id, dlc, data)

                entry = {
                    'timestamp': timestamp,
                    'arbitration_id': hex(arbitration_id).replace('0x', '').upper(),
                    'dlc': dlc,
                    'data': self.format_can_data(data)
                }

                # A delimiter message closes the frame being collected and
                # opens the next one; everything else is appended as it comes.
                if self.is_frame_start_message(arbitration_id, data):
                    if self.frame_active and self.current_frame:
                        try:
                            self.frame_queue.put_nowait(self.current_frame.copy())
                            with self.stats_lock:
                                self.frames_collected += 1
                        except queue.Full:
                            print("⚠️ Frame queue full - dropping frame")

                    self.frame_active = True
                    self.current_frame.clear()
                    self.current_frame.append(entry)
                elif self.frame_active:
                    self.current_frame.append(entry)

        except KeyboardInterrupt:
            print("🛑 User interrupted CAN collection")
        except Exception as e:
            print(f"❌ Error in CAN collection: {e}")
        finally:
            print("🔚 CAN message collection stopped")

    def log_to_csv(self, timestamp: float, arbitration_id: int, dlc: int, data: bytes):
        """Log CAN message to CSV file (preserves original logging functionality)"""
        try:
            if self._raw_csv_writer is None:
                self._raw_csv_handle = open(self.csv_filename, 'a', newline='')
                self._raw_csv_writer = csv.writer(self._raw_csv_handle)
            arbitration_id_str = hex(arbitration_id).replace('0x','').upper()
            self._raw_csv_writer.writerow(
                [timestamp, arbitration_id_str, dlc, self.format_can_data(data)])
        except Exception as e:
            print(f"❌ CSV logging error: {e}")

    def parse_frame_thread(self):
        """
        Frame parser thread - processes complete frames.
        This is where you implement your specific frame processing logic.
        Each frame contains a list of CAN messages that were collected
        between frame delimiter messages.
        """
       # print("🧠 Starting frame parser thread...")

        while self.running.is_set():
            try:
                # Get complete frame with timeout
                # frame_data = frame_data[] + frame_data
                frame_data = self.frame_queue.get(timeout=0.1)
                #print("........Printing frame data.........")
                #print(frame_data)

                # Process the complete frame
                self.process_complete_frame(frame_data)

                with self.stats_lock:
                    self.frames_processed += 1

            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ Frame parser error: {e}")

       # print(f"🏁 Frame parser thread stopped. Processed {self.frames_processed} frames.")

    def process_complete_frame(self, frame_data):
        """
        Process a complete frame of CAN messages.
        Args:
            frame_data: List of CAN message dictionaries, each containing:
                - timestamp: Message timestamp
                - arbitration_id: CAN ID as hex string
                - dlc: Data length code
                - data: Data bytes as hex string
        Returns:
            dict: Packed JSON object containing parsed data from all parsing functions
        """
        # Debug: Check what we actually received
        #print(f"\nDEBUG: frame_data type: {type(frame_data)}")
        #print(f"DEBUG: frame_data content: {frame_data}")

        if not frame_data:
            return {}

        # Handle case where frame_data might not be a list
        if not isinstance(frame_data, list):
            print(f"ERROR: Expected list, got {type(frame_data)}")
            return {}

        # Check if the list contains dictionaries
        if not all(isinstance(item, dict) for item in frame_data):
            print(f"ERROR: Not all items in frame_data are dictionaries")
            return {}

        # print(f"\n📋 Processing frame with {len(frame_data)} CAN messages")

        # Initialize the result dictionary
        result = {
            "Dynamic_obj": {},
            "Static_obj": {},
            "Cluster_obj": {},
            "HIL_Target1": {},
            "HIL_Target2": {},
            "ego_speed": {},
            "Alarm_thresh": {},
            "Alarm_status": {},
            "Point_cloud": {},
            "Tracked_obj": {},
            "Object_counts": {},
            # All three slots of each kind, and all four HIL targets
            "Dynamic_objects": {},
            "Static_objects": {},
            "HIL_Targets": {},
            "Time_msg": {},
            "Vehicle": {}
        }

        try:
            # Basic frame analysis with error checking
            if len(frame_data) > 0 and 'timestamp' in frame_data[0]:
                frame_start_time = frame_data[0]['timestamp']
                frame_end_time = frame_data[-1]['timestamp']
                frame_duration = frame_end_time - frame_start_time
            else:
                # print("WARNING: Cannot extract frame timing information")
                frame_duration = 0

            # Extract unique CAN IDs in frame with error checking
            unique_ids = set()
            for msg in frame_data:
                if isinstance(msg, dict) and 'arbitration_id' in msg:
                    unique_ids.add(msg['arbitration_id'])

            # print(f" 🆔 Unique CAN IDs: {sorted(unique_ids)}")
            # print(f" 📊 Message count: {len(frame_data)}")

            # Process messages by CAN ID
            id_counts = {}
            radar_raw_buffer = ""

            # Frames carrying no radar payload (no C1/D1/D2/D3) still need
            # these defined, otherwise the whole frame is dropped.
            Detect_Points = []
            Track_Objects = []
            Cluster_Objects = []

            for message in frame_data:
                if not isinstance(message, dict):
                    #print(f"WARNING: Skipping non-dict message: {message}")
                    continue

                # Check if required keys exist
                required_keys = ['arbitration_id', 'dlc', 'data', 'timestamp']
                if not all(key in message for key in required_keys):
                    # print(f"WARNING: Message missing required keys: {message}")
                    continue

                can_id = message['arbitration_id']
                dlc = message['dlc']
                data = message['data']
                timestamp = message['timestamp']

                # Count messages per ID
                id_counts[can_id] = id_counts.get(can_id, 0) + 1

                # Process based on CAN ID
                # Three moving and three stationary object slots
                if can_id in ('510', '511', '512', '590', '591', '592'):
                    slot = int(can_id[2], 16) + 1
                    kind = 'Dynamic' if can_id[1] == '1' else 'Static'
                    parsed_data = self.parse_object_signal(data, can_id)
                    if parsed_data:
                        x_val, y_val, Vx_val, Vy_val = parsed_data
                        payload = {"slot": slot, "x_val": x_val, "y_val": y_val,
                                   "Vx_val": Vx_val, "Vy_val": Vy_val}
                        result[f"{kind}_objects"][slot] = payload
                        if slot == 1:
                            # Closest object keeps its original key so existing
                            # consumers and the wide CSV are unaffected.
                            result[f"{kind}_obj"] = {
                                "x_val": x_val, "y_val": y_val,
                                "Vx_val": Vx_val, "Vy_val": Vy_val}

                elif can_id in ('100', '101', '102', '103'):
                    target = int(can_id, 16) - 0xFF   # 0x100 -> 1
                    parsed_data = self.parse_hil_target(data, can_id)
                    if parsed_data:
                        result["HIL_Targets"][target] = parsed_data
                        if target in (1, 2):
                            result[f"HIL_Target{target}"] = parsed_data

                elif can_id == '3F5':
                    parsed_data = self.parse_3F5(data)
                    if parsed_data:
                        result["ego_speed"] = {"ego_speed": parsed_data}

                elif can_id == '3F7':
                    parsed_data = self.parse_yaw(data)
                    if parsed_data:
                        result["Vehicle"] = parsed_data

                elif can_id == '6F1':
                    parsed_data = self.parse_0x6F1_Alarm_distance_Msg(data)
                    if parsed_data:
                        Level_1_Sta, Level_2_Sta, Level_3_Sta, Level_1_Mov, Level_2_Mov, Level_3_Mov = parsed_data
                        result["Alarm_thresh"] = {
                            "Level_1_Sta": Level_1_Sta,
                            "Level_2_Sta": Level_2_Sta,
                            "Level_3_Sta": Level_3_Sta,
                            "Level_1_Mov": Level_1_Mov,
                            "Level_2_Mov": Level_2_Mov,
                            "Level_3_Mov": Level_3_Mov
                        }

                elif can_id == '6F0':
                    parsed_data = self.parse_time_msg(data)
                    if parsed_data:
                        result["Time_msg"] = parsed_data

                elif can_id == '501':
                    parsed_data = self.parse_object_count(data, '501')
                    if parsed_data:
                        result["Object_counts"]["dynamic"] = parsed_data["count"]
                        result["Object_counts"]["dynamic_cycle"] = parsed_data["cycle_count"]

                elif can_id == '581':
                    parsed_data = self.parse_object_count(data, '581')
                    if parsed_data:
                        result["Object_counts"]["static"] = parsed_data["count"]
                        result["Object_counts"]["static_cycle"] = parsed_data["cycle_count"]

                elif can_id == '1':
                    parsed_data = self.parse_0X001_Alarm_Info(data)
                    if parsed_data:
                        result["Alarm_status"] = {"Alarm_type": parsed_data}

                elif can_id == 'C1' or can_id == 'D1' or can_id == 'D2' or  can_id == 'D3':
                    radar_raw_buffer += data
                    radar_raw_buffer += " "

            # Process radar frame data
            if radar_raw_buffer.strip():
                if DEBUG:
                    print(radar_raw_buffer)

                parsed_data = self.parse_frame_data(radar_raw_buffer)
                if parsed_data:
                    if len(parsed_data) == 2:
                        Detect_Points, Track_Objects = parsed_data
                    else:
                        Detect_Points, Track_Objects, Cluster_Objects = parsed_data
            # print(f" 📈 Message distribution: {dict(sorted(id_counts.items()))}")
            ##print(" ✅ Frame processing completed\n")
            
            # Assign a stable identity to each track. T_Y is longitudinal and
            # T_X lateral, which is the order the tracker expects.
            self._frame_counter += 1
            frame_timestamp = frame_data[0]['timestamp'] if frame_data else None

            # Same for the CAN object slots, in slot order
            for key, tracker in self.object_trackers.items():
                slots = sorted((result.get(key) or {}).items())
                if not slots:
                    continue
                ids = tracker.update(
                    [(float(p['x_val']), float(p['y_val']),
                      float(p['Vx_val']), float(p['Vy_val'])) for _, p in slots],
                    frame=self._frame_counter, timestamp=frame_timestamp)
                for (_, payload), assigned in zip(slots, ids):
                    payload['assoc_id'] = assigned
            assigned = self.tracker.update(
                [(t['T_Y'], t['T_X'], t.get('T_Vy', 0.0), t.get('T_Vx', 0.0))
                 for t in Track_Objects],
                frame=self._frame_counter,
                timestamp=frame_timestamp,
            )
            for track, track_id in zip(Track_Objects, assigned):
                track['T_ID'] = track_id

            result["Point_cloud"] = {"Detect_Points": Detect_Points}
            result["Tracked_obj"] = {"Track_Objects": Track_Objects}
      
            # NEW: expose clusters parsed from D2 / TLV_TYPE 2
            if Cluster_Objects:
                result["Cluster_obj"] = {"Clusters": Cluster_Objects}
                
            #print(result)

            # Send data to visualizer via socket
            self.send_to_visualizer(result)

            # Log parsed data asynchronously (NEW - Added after socket send)
            self.log_parsed_data_async(result, frame_timestamp)

            # Return the packed JSON object
            return result

        except Exception as e:
            print(f"ERROR in process_complete_frame: {e}")
            import traceback
            traceback.print_exc()
            return {}

    # ------------------------------------------------------------------
    # CAN signal decoding  (Starkenn Radar DBC V6)
    # ------------------------------------------------------------------
    # Signals are extracted by bit offset and length straight from the DBC
    # rather than hand-rolled per message. The generic extractor was checked
    # against the previously verified object layout over random payloads.

    @staticmethod
    def _bits(data, start_bit, length):
        """Unsigned little-endian (Intel) signal extraction, as the DBC uses."""
        value = 0
        for index in range(length):
            bit = start_bit + index
            byte_index = bit // 8
            if byte_index < len(data) and (data[byte_index] >> (bit % 8)) & 1:
                value |= 1 << index
        return value

    @staticmethod
    def _to_bytes(hex_string):
        return [int(token, 16) for token in hex_string.split()]

    # Shared object layout: 0x510/0x511/0x512, 0x590/0x591/0x592 and the HIL
    # targets all use it. Bits 47-63 are "Object_reserve_*" in the DBC - named
    # reserve, carrying nothing today.
    OBJECT_SIGNALS = {
        'x': (0, 13, 0.032, -60.8),      # longitudinal distance, m
        'y': (13, 12, 0.064, -131.072),  # lateral distance, m
        'vx': (25, 11, 0.1, -102.4),     # longitudinal speed, m/s
        'vy': (36, 11, 0.1, -102.4),     # lateral speed, m/s
    }

    def parse_object_signal(self, hex_string, label=""):
        """Decode the shared object layout. Returns (x, y, Vx, Vy) or None."""
        try:
            data = self._to_bytes(hex_string)
            if len(data) < 6:
                raise ValueError(f"need 6 bytes, got {len(data)}")
            out = []
            for name in ('x', 'y', 'vx', 'vy'):
                start, length, scale, offset = self.OBJECT_SIGNALS[name]
                out.append(format(scale * self._bits(data, start, length) + offset, '.4f'))
            return tuple(out)
        except Exception as e:
            print(f"Error parsing {label}:{hex_string}: {e}")
            return None

    def parse_590(self, hex_string):
        """Closest stationary object."""
        return self.parse_object_signal(hex_string, "590")

    def parse_510(self, hex_string):
        """Closest moving object."""
        return self.parse_object_signal(hex_string, "510")

    def parse_100(self, hex_string):
        """HIL ground-truth target 1."""
        return self.parse_object_signal(hex_string, "100")

    def parse_101(self, hex_string):
        """HIL ground-truth target 2."""
        return self.parse_object_signal(hex_string, "101")

    # DBC: "1-Pedestrian, 2-Cycle, 3-Car, 4-Truck"
    TARGET_TYPES = {0: "unknown", 1: "pedestrian", 2: "cycle", 3: "car", 4: "truck"}
    # DBC: "0 - Receding Target, 1- Oncoming Target"
    TARGET_TRAJECTORIES = {0: "receding", 1: "oncoming"}

    def parse_hil_target(self, hex_string, label=""):
        """HIL target: object layout plus classification, trajectory and RCS.

        These attributes come from the HIL rig, so they are ground truth about
        the target, not radar output. The radar itself reports no class.
        """
        base = self.parse_object_signal(hex_string, label)
        if base is None:
            return None
        try:
            data = self._to_bytes(hex_string)
            type_code = self._bits(data, 47, 4)
            trajectory_code = self._bits(data, 51, 2)
            payload = {
                "x_val": base[0], "y_val": base[1],
                "Vx_val": base[2], "Vy_val": base[3],
                "type_code": type_code,
                "type": self.TARGET_TYPES.get(type_code, f"code {type_code}"),
                "trajectory": self.TARGET_TRAJECTORIES.get(
                    trajectory_code, f"code {trajectory_code}"),
                "rcs_dbsm": self._bits(data, 53, 6),
            }
            return payload
        except Exception as e:
            print(f"Error parsing {label} attributes {hex_string}: {e}")
            return None

    def parse_3F5(self, hex_string):
        """Ego speed. DBC Vehicle_Data_Speed: 15 bits, 0.01, unit m/s."""
        try:
            data = self._to_bytes(hex_string)
            if len(data) < 2:
                raise ValueError("need 2 bytes")
            return format(0.01 * self._bits(data, 0, 15), '.2f')
        except Exception as e:
            print(f"Error parsing 3F5 {hex_string}: {e}")
            return None

    def parse_yaw(self, hex_string):
        """Ego yaw rate. DBC: sign bit 35, magnitude bits 36-49 at 0.01."""
        try:
            data = self._to_bytes(hex_string)
            if len(data) < 7:
                raise ValueError("need 7 bytes")
            magnitude = 0.01 * self._bits(data, 36, 14)
            sign = self._bits(data, 35, 1)
            return {"yaw_rate": round(-magnitude if sign else magnitude, 3),
                    "yaw_sign": sign}
        except Exception as e:
            print(f"Error parsing 3F7 {hex_string}: {e}")
            return None

    # DBC: Alarm_distance_Msg, 0.5 m per LSB, range 0..127.5 m
    ALARM_THRESH_SCALE = 0.5

    def parse_0x6F1_Alarm_distance_Msg(self, hex_string):
        """Alarm distance thresholds in metres: 3 static levels, then 3 moving.

        Byte 3 is unused; the DBC places the moving levels at bytes 4-6.
        """
        try:
            data = self._to_bytes(hex_string)
            if len(data) < 7:
                raise ValueError("need 7 bytes")
            return [round(data[i] * self.ALARM_THRESH_SCALE, 1)
                    for i in (0, 1, 2, 4, 5, 6)]
        except Exception as e:
            print(f"Error parsing 6F1 {hex_string}: {e}")
            return None

    def parse_time_msg(self, hex_string):
        """0x6F0 Time_Msg: time to collision, post-encroachment time, and the
        index of the object those refer to."""
        try:
            data = self._to_bytes(hex_string)
            if len(data) < 8:
                raise ValueError("need 8 bytes")
            return {
                "ttc": round(0.01 * self._bits(data, 0, 16), 2),
                "pet": round(0.01 * self._bits(data, 32, 16), 2),
                "relevant_object": self._bits(data, 56, 4),
            }
        except Exception as e:
            print(f"Error parsing 6F0 {hex_string}: {e}")
            return None

    def parse_0X001_Alarm_Info(self, hex_string):
        """Active alarm type. 2 bits, values 0-3."""
        try:
            data = self._to_bytes(hex_string)
            if not data:
                raise ValueError("empty payload")
            return data[0] & 0b11
        except Exception as e:
            print(f"Error parsing 001 {hex_string}: {e}")
            return None

    def parse_object_count(self, hex_string, label=""):
        """Object count messages: count, cycle time and a free-running cycle
        counter. The counter is what reveals a dropped radar cycle."""
        try:
            data = self._to_bytes(hex_string)
            if not data:
                raise ValueError("empty payload")
            return {
                "count": self._bits(data, 0, 8),
                "cycle_time": self._bits(data, 8, 8) if len(data) >= 2 else None,
                "cycle_count": self._bits(data, 32, 32) if len(data) >= 8 else None,
            }
        except Exception as e:
            print(f"Error parsing {label} {hex_string}: {e}")
            return None

    # ------------------------------------------------------------------
    # mmWave frame decoding
    # ------------------------------------------------------------------
    # Packet layout, after the 8-byte barker code that opens the frame:
    #   header    CRC(4) len(4) platform(4) frameNum(4) cpuTime(4)
    #             numDetObj(4) numTLV(4) subFrame(4)          = 32 bytes
    #   each TLV  type(4) length(4) then a type-specific body
    HEADER_BYTES = 40           # barker included
    TLV_HEADER_BYTES = 8
    POINT_RECORD_BYTES = 14     # doppler(2) peak(2) x(2) y(2) spare(6)
    CLUSTER_RECORD_BYTES = 8    # x(2) y(2) xSize(2) ySize(2)
    TRACK_RECORD_BYTES = 12     # x(2) y(2) vx(2) vy(2) spare(4)

    TLV_POINT_CLOUD = 1
    TLV_CLUSTERS = 2
    TLV_TRACKS = 3

    # Tracks reported closer than this are padding in unused track slots
    MIN_VALID_TRACK_RANGE_M = 0.8

    @staticmethod
    def _u16(buf, offset):
        return buf[offset] | (buf[offset + 1] << 8)

    @staticmethod
    def _s16(buf, offset):
        value = buf[offset] | (buf[offset + 1] << 8)
        return value - 65536 if value > 32767 else value

    @staticmethod
    def _u32(buf, offset):
        return (buf[offset] | (buf[offset + 1] << 8)
                | (buf[offset + 2] << 16) | (buf[offset + 3] << 24))

    def parse_frame_data(self, data_hex):
        """Decode one mmWave packet into detections, clusters and tracks.

        Every read is bounds-checked: a truncated packet yields whatever was
        decoded before the data ran out rather than raising.
        """
        Detect_Points = []
        Track_Objects = []
        Cluster_Objects = []

        try:
            buf = bytes(int(token, 16) for token in data_hex.split())
        except ValueError as e:
            if DEBUG:
                print(f"⚠️ Non-hex byte in radar buffer: {e}")
            return Detect_Points, Track_Objects, Cluster_Objects

        # A frame holding only the delimiter (normal at the end of a
        # recording) or a truncated payload has nothing to decode.
        if len(buf) < self.HEADER_BYTES:
            if DEBUG:
                print(f"⚠️ Radar buffer too short to parse: {len(buf)} bytes")
            return Detect_Points, Track_Objects, Cluster_Objects

        num_tlv = self._u32(buf, 32)
        idx = self.HEADER_BYTES

        for _ in range(num_tlv):
            if idx + self.TLV_HEADER_BYTES > len(buf):
                break
            tlv_type = self._u32(buf, idx)
            tlv_len = self._u32(buf, idx + 4)
            idx += self.TLV_HEADER_BYTES

            if tlv_type == self.TLV_POINT_CLOUD:
                idx = self._parse_point_cloud_tlv(buf, idx, Detect_Points)
            elif tlv_type == self.TLV_CLUSTERS:
                idx = self._parse_cluster_tlv(buf, idx, Cluster_Objects)
            elif tlv_type == self.TLV_TRACKS:
                idx = self._parse_track_tlv(buf, idx, Track_Objects)
            else:
                # Unknown TLV: skip its body using the declared length.
                if DEBUG:
                    print(f"⚠️ Skipping unknown TLV type {tlv_type} ({tlv_len} bytes)")
                idx += tlv_len

            if idx is None or idx > len(buf):
                break

        if DEBUG:
            print(f"points={len(Detect_Points)} tracks={len(Track_Objects)} "
                  f"clusters={len(Cluster_Objects)}")

        return (
            sorted(Detect_Points, key=lambda point: point['D_Y']),
            sorted(Track_Objects, key=lambda track: track['T_Y']),
            sorted(Cluster_Objects, key=lambda cluster: cluster['C_Y']),
        )

    def _parse_point_cloud_tlv(self, buf, idx, out):
        """TLV 1 - detected point cloud."""
        if idx + 4 > len(buf):
            return idx
        count = self._u16(buf, idx)
        q_format = self._u16(buf, idx + 2)
        idx += 4
        scale = float(1 << q_format)

        for _ in range(count):
            if idx + self.POINT_RECORD_BYTES > len(buf):
                break
            out.append({
                'D_X': self._s16(buf, idx + 4) / scale,
                'D_Y': self._s16(buf, idx + 6) / scale,
                'Doppler': self._s16(buf, idx) / scale,
                'Peak': self._u16(buf, idx + 2),
            })
            idx += self.POINT_RECORD_BYTES
        return idx

    def _parse_cluster_tlv(self, buf, idx, out):
        """TLV 2 - clustered objects, with their extent."""
        if idx + 4 > len(buf):
            return idx
        count = self._u16(buf, idx)
        q_format = self._u16(buf, idx + 2)
        idx += 4
        scale = float(1 << q_format)

        for _ in range(count):
            if idx + self.CLUSTER_RECORD_BYTES > len(buf):
                break
            out.append({
                'C_X': self._s16(buf, idx) / scale,
                'C_Y': self._s16(buf, idx + 2) / scale,
                # Extent is reported as a half-size about the centre
                'C_SX': abs(self._s16(buf, idx + 4)) / scale,
                'C_SY': abs(self._s16(buf, idx + 6)) / scale,
            })
            idx += self.CLUSTER_RECORD_BYTES
        return idx

    def _parse_track_tlv(self, buf, idx, out):
        """TLV 3 - tracked objects."""
        if idx + 4 > len(buf):
            return idx
        count = self._u16(buf, idx)
        q_format = self._u16(buf, idx + 2)
        idx += 4
        scale = float(1 << q_format)

        for _ in range(count):
            if idx + self.TRACK_RECORD_BYTES > len(buf):
                break
            track_y = self._s16(buf, idx + 2) / scale
            # Unused track slots report ~zero range; skip just that slot
            # instead of abandoning the tracks that follow it.
            if track_y < self.MIN_VALID_TRACK_RANGE_M:
                idx += self.TRACK_RECORD_BYTES
                continue
            out.append({
                'T_X': self._s16(buf, idx) / scale,
                'T_Y': track_y,
                'T_Vx': self._s16(buf, idx + 4) / scale,
                'T_Vy': self._s16(buf, idx + 6) / scale,
            })
            idx += self.TRACK_RECORD_BYTES
        return idx

    def start_processing(self) -> bool:
        """Start CAN collection and frame processing threads"""
        # Setup socket server first
        if not self.setup_socket_server():
            print("❌ Failed to start - Socket server not available")
            return False

        if not self.setup_can_bus():
            print("❌ Failed to start - CAN bus not available")
            return False

        # Create and start threads
        self.can_thread = threading.Thread(
            target=self.collect_can_messages,
            name="CANCollectionThread",
            daemon=True
        )

        self.parser_thread = threading.Thread(
            target=self.parse_frame_thread,
            name="FrameParserThread",
            daemon=True
        )

        # NEW: Start parsed CSV writer thread
        self.parsed_csv_thread = threading.Thread(
            target=self.parsed_csv_writer_thread,
            name="ParsedCSVWriterThread",
            daemon=True
        )

        # Optionally hold the source until the visualizer is attached, so a
        # finite replay is not half spent before anything is listening.
        if self.wait_for_visualizer > 0:
            print(f"⏳ Waiting up to {self.wait_for_visualizer:.0f}s for the visualizer...")
            deadline = time.monotonic() + self.wait_for_visualizer
            while self.client_connection is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if self.client_connection is None:
                print("⚠️ Visualizer did not connect; starting anyway")

        # Start all threads
        self.can_thread.start()
        self.parser_thread.start()
        self.parsed_csv_thread.start()

        # print("🚀 CAN frame processor started successfully!")
        # print(f"📝 Raw CSV logging: {self.csv_filename}")
        # print(f"📊 Parsed CSV logging: {self.parsed_csv_filename}")
        return True

    def stop_processing(self):
        """Stop all processing threads gracefully"""
        print("🛑 Stopping CAN frame processor...")
        self.running.clear()
        self.socket_running = False

        # Wait for threads to complete
        if hasattr(self, 'can_thread'):
            self.can_thread.join(timeout=3.0)
        if hasattr(self, 'parser_thread'):
            self.parser_thread.join(timeout=3.0)
        if hasattr(self, 'parsed_csv_thread'):
            self.parsed_csv_thread.join(timeout=3.0)
        if hasattr(self, 'socket_thread'):
            self.socket_thread.join(timeout=3.0)

        # Flush and close log files
        for handle in (self._raw_csv_handle, self._parsed_csv_handle,
                       self._frames_csv_handle, self._objects_csv_handle):
            if handle:
                try:
                    handle.flush()
                    handle.close()
                except Exception as e:
                    print(f"⚠️ Error closing log file: {e}")
        self._raw_csv_handle = self._raw_csv_writer = None
        self._parsed_csv_handle = self._parsed_csv_writer = None
        self._frames_csv_handle = self._frames_csv_writer = None
        self._objects_csv_handle = self._objects_csv_writer = None

        # Close connections
        if self.client_connection:
            self.client_connection.close()
        if self.server_socket:
            self.server_socket.close()

        # Close CAN bus
        if self.bus:
            self.bus.shutdown()

        print("🔌 CAN bus disconnected")
        print("✅ CAN frame processor stopped gracefully")

    def get_statistics(self) -> Dict[str, Any]:
        """Get processing statistics"""
        elapsed_time = time.perf_counter() - self.start_time
        with self.stats_lock:
            stats = {
                'messages_received': self.messages_received,
                'frames_collected': self.frames_collected,
                'frames_processed': self.frames_processed,
                'parsed_logs_written': self.parsed_logs_written,
                'elapsed_time': elapsed_time,
                'message_rate': self.messages_received / elapsed_time if elapsed_time > 0 else 0,
                'frame_rate': self.frames_processed / elapsed_time if elapsed_time > 0 else 0,
                'frame_queue_size': self.frame_queue.qsize(),
                'parsed_queue_size': self.parsed_data_queue.qsize(),
                'avg_frame_size': self.messages_received / self.frames_collected if self.frames_collected > 0 else 0
            }
        return stats

def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Collect CAN frames from hardware, a virtual bus, or a logged CSV."
    )
    parser.add_argument(
        "--source", default=None,
        help="kvaser | virtual | replay:<log.csv> | any python-can interface "
             "(default: $ORDLAV_SOURCE, else kvaser on Windows/Linux, "
             "virtual elsewhere)")
    parser.add_argument("--channel", default="0", help="CAN channel (default: 0)")
    parser.add_argument("--bitrate", type=int, default=500000,
                        help="CAN bitrate (default: 500000)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Replay speed multiplier; 0 = as fast as possible "
                             "(default: 1.0)")
    parser.add_argument("--loop", action="store_true",
                        help="Restart a replayed log when it ends")
    parser.add_argument("--no-raw-log", action="store_true",
                        help="Skip the raw CAN CSV (useful when replaying one)")
    parser.add_argument("--wait-for-visualizer", type=float, default=0.0,
                        metavar="SECONDS",
                        help="Hold the source until the visualizer connects, "
                             "up to SECONDS (useful when replaying a log)")
    parser.add_argument("--stats-interval", type=float, default=10.0,
                        help="Seconds between statistics lines (default: 10)")
    return parser.parse_args(argv)


def main(argv=None):
    """
    Main function - replaces your original CAN logging loop
    """
    args = parse_args(argv)

    print("=== CAN Frame Processor ===")
    print("High-performance CAN message collection with frame-based processing\n")

    csv_filename = datetime.now()
    csv_filename=csv_filename.strftime('%H_%M_%m_%d_%Y.csv')
    CSV_FILE = csv_filename

    # Create processor with your configuration
    processor = CANFrameProcessor(
        channel=args.channel,
        bitrate=args.bitrate,
        csv_filename=CSV_FILE,
        source=args.source,
        replay_speed=args.speed,
        replay_loop=args.loop,
        raw_log=not args.no_raw_log,
        wait_for_visualizer=args.wait_for_visualizer,
    )

    try:
        # Start processing
        if not processor.start_processing():
            print("❌ Failed to start CAN processing")
            return

        print("✅ CAN frame processing is running...")
        print(f"📝 Raw CAN log:    {'(disabled)' if not processor.raw_log else processor.csv_filename}")
        print(f"📊 Parsed CSV log: {processor.parsed_csv_filename}")
        print("🛑 Press Ctrl+C to stop\n")

        # Monitor and display statistics
        while True:
            # A replayed log ends; wait for the queues to drain, then stop.
            if processor.source_exhausted.wait(timeout=args.stats_interval):
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline and (
                        processor.frame_queue.qsize()
                        or processor.parsed_data_queue.qsize()):
                    time.sleep(0.05)
                print("🏁 End of replayed log reached")
                break
            # stats = processor.get_statistics()
            # print(f"\n📈 --- Performance Statistics ---")
            # print(f" Runtime: {stats['elapsed_time']:.1f} seconds")
            # print(f" CAN Messages: {stats['messages_received']:,}")
            # print(f" Frames Collected: {stats['frames_collected']:,}")
            # print(f" Frames Processed: {stats['frames_processed']:,}")
            # print(f" Parsed Logs: {stats['parsed_logs_written']:,}")
            # print(f" Message Rate: {stats['message_rate']:.1f} msg/s")
            # print(f" Frame Rate: {stats['frame_rate']:.2f} frames/s")
            # print(f" Avg Frame Size: {stats['avg_frame_size']:.1f} messages/frame")
            # print(f" Queue Sizes - Frame: {stats['frame_queue_size']}, Parsed: {stats['parsed_queue_size']}")

    except KeyboardInterrupt:
        print("\n🛑 User interrupted - shutting down gracefully...")
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
    finally:
        processor.stop_processing()

        # Display final statistics
        final_stats = processor.get_statistics()
        print(f"\n🏁 === Final Statistics ===")
        print(f"  Runtime:          {final_stats['elapsed_time']:.1f} s")
        print(f"  CAN messages:     {final_stats['messages_received']:,}")
        print(f"  Frames collected: {final_stats['frames_collected']:,}")
        print(f"  Frames processed: {final_stats['frames_processed']:,}")
        print(f"  Parsed log rows:  {final_stats['parsed_logs_written']:,}")
        print(f"  Message rate:     {final_stats['message_rate']:.1f} msg/s")
        print(f"  Frame rate:       {final_stats['frame_rate']:.2f} frames/s")
        if processor.raw_log:
            print(f"  Raw CSV:          {processor.csv_filename}")
        print(f"  Parsed CSV:       {processor.parsed_csv_filename}")
        # print(f"\n🏁 === Final Statistics ===")
        # print(f" Total Runtime: {final_stats['elapsed_time']:.1f} seconds")
        # print(f" Total CAN Messages: {final_stats['messages_received']:,}")
        # print(f" Total Frames Processed: {final_stats['frames_processed']:,}")
        # print(f" Total Parsed Logs: {final_stats['parsed_logs_written']:,}")
        # print(f" Average Message Rate: {final_stats['message_rate']:.1f} msg/s")
        # print(f" Average Frame Rate: {final_stats['frame_rate']:.2f} frames/s")
        # print(" Raw CSV file: " + processor.csv_filename)
        # print(" Parsed CSV file: " + processor.parsed_csv_filename)
        # print("\n✅ Processing completed successfully!")

if __name__ == "__main__":
    # IMPORTANT: Update these constants for your system
    # Replace with your actual frame header CAN ID
    FRAME_START_DATA = "02 01 04 03 06 05 08 07"  # Your actual frame delimiter data

    # print("🔧 Configuration Check:")
    # print(f" Frame Header CAN ID: 0x{OR_MRR_DATA_HEADER:X}")
    # print(f" Frame Start Data Pattern: {FRAME_START_DATA}")
    # print(" ⚠️ Make sure these values match your system requirements!")
    # print()

    # Run the main processing loop
    main()
