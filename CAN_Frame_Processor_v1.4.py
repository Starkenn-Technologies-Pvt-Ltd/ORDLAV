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

import sys
if sys.platform.startswith("win"):
    sys.stdout.reconfigure(encoding="utf-8")
# CAN Configuration Constants
# TODO: Update these values for your specific system
#OR_MRR_DATA_HEADER = 0x123 # Replace with your actual CAN ID for frame header
OR_MRR_DATA_HEADER = 0xC1
OR_MRR_POINT_CLOUD = 0xD1
OR_MRR_TRACKERS = 0xD3
OR_MRR_PADDING = 0xE1
OR_FCWS_MOVING_OBJ = 0x510
OR_FCWS_STATIC_OBJ = 0x590
OR_ALARM_STATUS = 0xA1
OR_FCWS_THRESH = 0x6F1
VEH_SPEED = 0x3F5
OR_ECU_ALARM = 0x1
HIL_100 =0x100
HIL_101 =0x101
HIL_102 =0x102

can_ids_to_filter = [OR_MRR_DATA_HEADER, OR_MRR_POINT_CLOUD, OR_MRR_TRACKERS, OR_FCWS_STATIC_OBJ, OR_FCWS_MOVING_OBJ, OR_ALARM_STATUS, OR_FCWS_THRESH, VEH_SPEED, OR_ECU_ALARM, HIL_100, HIL_101]

FRAME_START_DATA = "02 01 04 03 06 05 08 07"  # Frame delimiter data pattern

class CANFrameProcessor:
    """
    High-performance CAN frame processor that collects CAN messages into frames
    based on specific header and data patterns, then processes complete frames
    in a separate thread for optimal performance.
    """

    def __init__(self, channel='0', bitrate=500000, csv_filename: str = None):
        # CAN bus configuration
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None

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
        self.parsed_csv_filename = "parsed_" + timestamp.strftime('%H_%M_%m_%d_%Y.csv')
        self.parsed_data_queue = queue.Queue(maxsize=1000)

        # Initialize parsed CSV with headers
        self._initialize_parsed_csv()

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
                'tracked_objects'
            ]

            with open(self.parsed_csv_filename, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(headers)

          #  print(f"✅ Initialized parsed CSV: {self.parsed_csv_filename}")

        except Exception as e:
            print(f"❌ Error initializing parsed CSV: {e}")

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

                # Keep connection alive
                while self.socket_running:
                    try:
                        # Send ping to check if client is still connected
                        self.client_connection.send(b'ping')
                        time.sleep(1)
                    except (BrokenPipeError, ConnectionResetError):
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
                    data.get('Tracked_obj', {}).get('Track_Objects', [])
                ]

                # Write to CSV
                with open(self.parsed_csv_filename, 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(row_data)

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
            self.bus = can.interface.Bus(
                channel=self.channel,
                bustype='kvaser',
                bitrate=self.bitrate,
                extended=False
            )
            self.bus.set_filters(filters)
            print(f"✅ CAN bus connected successfully")
            return True
        except Exception as e:
            print(f"❌ Failed to initialize CAN bus: {e}")
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
                self.log_to_csv(timestamp, arbitration_id, dlc, data)

                # Step 1: Check if arbitration_id equals OR_MRR_DATA_HEADER
                if arbitration_id == OR_MRR_DATA_HEADER:
                    # Step 1.a: Check if data equals frame start pattern
                    formatted_data = self.format_can_data(data)
                    if formatted_data == FRAME_START_DATA:
                        # Step 1.a.i: Frame delimiter found
                        if self.frame_active:
                            # End of current frame - send to parser thread
                            if len(self.current_frame) > 0:
                                try:
                                    # Step 2: Send complete frame to parse_frame_thread
                                    self.frame_queue.put_nowait(self.current_frame.copy())
                                    with self.stats_lock:
                                        self.frames_collected += 1
                                    # print(f"📦 Frame completed with {len(self.current_frame)} messages")
                                except queue.Full:
                                    print("⚠️ Frame queue full - dropping frame")

                            # Start new frame
                            self.current_frame.clear()
                            can_message_entry = {
                                'timestamp': timestamp,
                                'arbitration_id': hex(arbitration_id).replace('0x','').upper(),
                                'dlc': dlc,
                                'data': self.format_can_data(data)
                            }
                            self.current_frame.append(can_message_entry)
                        else:
                            # First frame start detected
                            self.frame_active = True
                            self.current_frame.clear()
                            # print("🔄 Frame collection started")
                            can_message_entry = {
                                'timestamp': timestamp,
                                'arbitration_id': hex(arbitration_id).replace('0x','').upper(),
                                'dlc': dlc,
                                'data': self.format_can_data(data)
                            }
                            self.current_frame.append(can_message_entry)
                    else:
                        if self.frame_active:
                            # Store message in frame with format [arbitration_id, dlc, data]
                            can_message_entry = {
                                'timestamp': timestamp,
                                'arbitration_id': hex(arbitration_id).replace('0x','').upper(),
                                'dlc': dlc,
                                'data': self.format_can_data(data)
                            }
                            self.current_frame.append(can_message_entry)

                # Step 1.b: Regular CAN message - add to frame if collecting
                else:
                    if self.frame_active:
                        # Store message in frame with format [arbitration_id, dlc, data]
                        can_message_entry = {
                            'timestamp': timestamp,
                            'arbitration_id': hex(arbitration_id).replace('0x','').upper(),
                            'dlc': dlc,
                            'data': self.format_can_data(data)
                        }
                        self.current_frame.append(can_message_entry)

        except KeyboardInterrupt:
            print("🛑 User interrupted CAN collection")
        except Exception as e:
            print(f"❌ Error in CAN collection: {e}")
        finally:
            print("🔚 CAN message collection stopped")

    def log_to_csv(self, timestamp: float, arbitration_id: int, dlc: int, data: bytes):
        """Log CAN message to CSV file (preserves original logging functionality)"""
        try:
            arbitration_id_str = hex(arbitration_id).replace('0x','').upper()
            data_str = self.format_can_data(data)
            with open(self.csv_filename, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([timestamp, arbitration_id_str, dlc, data_str])
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
            "HIL_Target1": {},
            "HIL_Target2": {},
            "ego_speed": {},
            "Alarm_thresh": {},
            "Alarm_status": {},
            "Point_cloud": {},
            "Tracked_obj": {}
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
                if can_id == '590':
                    parsed_data = self.parse_590(data)
                    if parsed_data:
                        x_val, y_val, Vx_val, Vy_val = parsed_data
                        result["Static_obj"] = {
                            "x_val": x_val,
                            "y_val": y_val,
                            "Vx_val": Vx_val,
                            "Vy_val": Vy_val
                        }

                elif can_id == '510':
                    parsed_data = self.parse_510(data)
                    if parsed_data:
                        x_val, y_val, Vx_val, Vy_val = parsed_data
                        result["Dynamic_obj"] = {
                            "x_val": x_val,
                            "y_val": y_val,
                            "Vx_val": Vx_val,
                            "Vy_val": Vy_val
                        }
                elif can_id == '100':
                    parsed_data = self.parse_100(data)
                    if parsed_data:
                        x_val, y_val, Vx_val, Vy_val = parsed_data
                        result["HIL_Target1"] = {
                            "x_val": x_val,
                            "y_val": y_val,
                            "Vx_val": Vx_val,
                            "Vy_val": Vy_val
                        }
                elif can_id == '101':
                    parsed_data = self.parse_101(data)
                    if parsed_data:
                        x_val, y_val, Vx_val, Vy_val = parsed_data
                        result["HIL_Target2"] = {
                            "x_val": x_val,
                            "y_val": y_val,
                            "Vx_val": Vx_val,
                            "Vy_val": Vy_val
                        }

                elif can_id == '3F5':
                    parsed_data = self.parse_3F5(data)
                    if parsed_data:
                        result["ego_speed"] = {"ego_speed": parsed_data}
                        #print(parsed_data)

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

                elif can_id == '1':
                    parsed_data = self.parse_0X001_Alarm_Info(data)
                    if parsed_data:
                        result["Alarm_status"] = {"Alarm_type": parsed_data}

                elif can_id == 'C1' or can_id == 'D1' or can_id == 'D3':
                    radar_raw_buffer += data
                    radar_raw_buffer += " "

            # Process radar frame data
            if radar_raw_buffer.strip():
                #print(radar_raw_buffer)
                parsed_data = self.parse_frame_data(radar_raw_buffer)
                if parsed_data:
                    Detect_Points, Track_Objects = parsed_data
                    result["Point_cloud"] = {"Detect_Points": Detect_Points}
                    result["Tracked_obj"] = {"Track_Objects": Track_Objects}

            # print(f" 📈 Message distribution: {dict(sorted(id_counts.items()))}")
            ##print(" ✅ Frame processing completed\n")
            # print(result)

            # Send data to visualizer via socket
            self.send_to_visualizer(result)

            # Log parsed data asynchronously (NEW - Added after socket send)
            frame_timestamp = frame_data[0]['timestamp'] if frame_data else None
            self.log_parsed_data_async(result, frame_timestamp)

            # Return the packed JSON object
            return result

        except Exception as e:
            print(f"ERROR in process_complete_frame: {e}")
            import traceback
            traceback.print_exc()
            return {}

    def parse_590(self,hex_string):
        try:
            bytes_list = hex_string.split()
            x_val = int(bytes_list[0], 16 ) | ((int(bytes_list[1], 16) & 0b00011111 ) << 8)
            y_val = ((int(bytes_list[1], 16) & 0b11100000) >> 5) | ((int(bytes_list[2], 16) & 0b11111111) << 3) | ((int(bytes_list[3], 16) & 0b00000001) << 11)
            Vx_val = ((int(bytes_list[3], 16) & 0b11111110) >> 1) | ((int(bytes_list[4], 16) & 0b00001111) << 7)
            Vy_val = ((int(bytes_list[4], 16) & 0b11110000) >> 4) | ((int(bytes_list[5], 16) & 0b01111111) << 4 )

            x_val = format(0.032 * x_val - 60.8, '.4f')
            y_val = format(0.064 * y_val - 131.072, '.4f')
            Vx_val = format(0.1 * Vx_val - 102.4, '.4f')
            Vy_val = format(0.1 * Vy_val - 102.4, '.4f')

            return x_val,y_val,Vx_val,Vy_val
        except Exception as e:
            print(f"Error parsing 590:{hex_string}: {e}")
            return None

    

    def parse_510(self,hex_string):
        try:
            bytes_list = hex_string.split()
            x_val = int(bytes_list[0], 16 ) | ((int(bytes_list[1], 16) & 0b00011111 ) << 8)
            y_val = ((int(bytes_list[1], 16) & 0b11100000) >> 5) | ((int(bytes_list[2], 16) & 0b11111111) << 3) | ((int(bytes_list[3], 16) & 0b00000001) << 11)
            Vx_val = ((int(bytes_list[3], 16) & 0b11111110) >> 1) | ((int(bytes_list[4], 16) & 0b00001111) << 7)
            Vy_val = ((int(bytes_list[4], 16) & 0b11110000) >> 4) | ((int(bytes_list[5], 16) & 0b01111111) << 4)

            x_val = format(0.032 * x_val - 60.8, '.4f')
            y_val = format(0.064 * y_val - 131.072, '.4f')
            Vx_val = format(0.1 * Vx_val - 102.4, '.4f')
            Vy_val = format(0.1 * Vy_val - 102.4, '.4f')

            return x_val,y_val,Vx_val,Vy_val
        except Exception as e:
            print(f"Error parsing 510:{hex_string}: {e}")
            return None

    def parse_100(self,hex_string):
        try:
            bytes_list = hex_string.split()

            x_val = int(bytes_list[0], 16 ) | ((int(bytes_list[1], 16) & 0b00011111 ) << 8)
        
            y_val = ((int(bytes_list[1], 16) & 0b11100000) >> 5) | ((int(bytes_list[2], 16) & 0b11111111) << 3) | ((int(bytes_list[3], 16) & 0b00000001) << 11)
            Vx_val = ((int(bytes_list[3], 16) & 0b11111110) >> 1) | ((int(bytes_list[4], 16) & 0b00001111) << 7)
            Vy_val = ((int(bytes_list[4], 16) & 0b11110000) >> 4) | ((int(bytes_list[5], 16) & 0b01111111) << 4 )

            x_val = format(0.032 * x_val - 60.8, '.4f')
            y_val = format(0.064 * y_val - 131.072, '.4f')
            Vx_val = format(0.1 * Vx_val - 102.4, '.4f')
            Vy_val = format(0.1 * Vy_val - 102.4, '.4f')
            return x_val,y_val,Vx_val,Vy_val
        except Exception as e:
            print(f"Error parsing 100:{hex_string}: {e}")
        return None

    def parse_101(self,hex_string):
        try:
            bytes_list = hex_string.split()

            x_val = int(bytes_list[0], 16 ) | ((int(bytes_list[1], 16) & 0b00011111 ) << 8)
        
            y_val = ((int(bytes_list[1], 16) & 0b11100000) >> 5) | ((int(bytes_list[2], 16) & 0b11111111) << 3) | ((int(bytes_list[3], 16) & 0b00000001) << 11)
            Vx_val = ((int(bytes_list[3], 16) & 0b11111110) >> 1) | ((int(bytes_list[4], 16) & 0b00001111) << 7)
            Vy_val = ((int(bytes_list[4], 16) & 0b11110000) >> 4) | ((int(bytes_list[5], 16) & 0b01111111) << 4 )

            x_val = format(0.032 * x_val - 60.8, '.4f')
            y_val = format(0.064 * y_val - 131.072, '.4f')
            Vx_val = format(0.1 * Vx_val - 102.4, '.4f')
            Vy_val = format(0.1 * Vy_val - 102.4, '.4f')
            return x_val,y_val,Vx_val,Vy_val
        except Exception as e:
            print(f"Error parsing 101:{hex_string}: {e}")
        return None

    def parse_3F5(self,hex_string):
        try:
            bytes_list = hex_string.split()
            ego_speed = int(bytes_list[0], 16) | ((int(bytes_list[1], 16) & 0b01111111)<<8)
            ego_speed = format(((float (format(0.01 * ego_speed + 0, '.4f')))) ,'.1f')
            # need to convert the required units
            return ego_speed
        except Exception as e:
            print(f"Error parsing 3f5{hex_string}: {e}")
            return None

    # Parsing Alarm distange messages
    def parse_0x6F1_Alarm_distance_Msg(self,hex_string):
        try:
            bytes_list=hex_string.split()
            Level_1_Sta=int("".join(map(str, bytes_list[0])),16)
            Level_2_Sta=int("".join(map(str, bytes_list[1])),16)
            Level_3_Sta=int("".join(map(str, bytes_list[2])),16)
            Level_1_Mov=int("".join(map(str, bytes_list[4])),16)
            Level_2_Mov=int("".join(map(str, bytes_list[5])),16)
            Level_3_Mov=int("".join(map(str, bytes_list[6])),16)

            return [Level_1_Sta,Level_2_Sta,Level_3_Sta,Level_1_Mov,Level_2_Mov,Level_3_Mov]
            # need to convert the required units
        except Exception as e:
            print(f"Error parsing {hex_string}: {e}")
            return None

    # Parse the Alarm info
    def parse_0X001_Alarm_Info(self,hex_string):
        bytes_list = hex_string.split()
        Alarm_type = int(bytes_list[0], 16) & 0b11111111
        #need to convert the required units
        return Alarm_type

    def parse_frame_data(self, data_hex):
        Detect_Points=[]
        Track_Objects=[]
        data_list = data_hex.split()
        idx = 0

        BARKER_CODE = data_list[idx:idx+8]
        idx += 8

        CRC = data_list[idx:idx+4]
        idx += 4

        MSG_LEN_bytes = data_list[idx:idx+4]
        MSG_LEN = int(MSG_LEN_bytes[0], 16) + (int(MSG_LEN_bytes[1], 16) << 8) + \
                  (int(MSG_LEN_bytes[2], 16) << 16) + (int(MSG_LEN_bytes[3], 16) << 24)
        idx += 4

        PLATFORM = data_list[idx:idx+4]
        idx += 4

        FRAME_NUM_BYTES = data_list[idx:idx+4]
        FRAME_NUM = int(FRAME_NUM_BYTES[0], 16) + (int(FRAME_NUM_BYTES[1], 16) << 8) + \
                    (int(FRAME_NUM_BYTES[2], 16) << 16) + (int(FRAME_NUM_BYTES[3], 16) << 24)
        idx += 4

        CPU_TIME = data_list[idx:idx+4]
        idx += 4

        NUM_DET_OBJ_BYTES = data_list[idx:idx+4]
        NUM_DET_OBJ = int(NUM_DET_OBJ_BYTES[0], 16) + (int(NUM_DET_OBJ_BYTES[1], 16) << 8) + \
                      (int(NUM_DET_OBJ_BYTES[2], 16) << 16) + (int(NUM_DET_OBJ_BYTES[3], 16) << 24)
        idx += 4

        NUM_OF_TLV_BYTES = data_list[idx:idx+4]
        NUM_OF_TLV = int(NUM_OF_TLV_BYTES[0], 16) + (int(NUM_OF_TLV_BYTES[1], 16) << 8) + \
                     (int(NUM_OF_TLV_BYTES[2], 16) << 16) + (int(NUM_OF_TLV_BYTES[3], 16) << 24)
        idx += 4

        idx += 4

        if NUM_OF_TLV == 1:
            TLV_TYPE_BYTES = data_list[idx:idx+4]
            TLV_TYPE = int(TLV_TYPE_BYTES[0], 16) + (int(TLV_TYPE_BYTES[1], 16) << 8) + \
                       (int(TLV_TYPE_BYTES[2], 16) << 16) + (int(TLV_TYPE_BYTES[3], 16) << 24)
            idx += 4

            if TLV_TYPE == 1:
                TLV_LEN_BYTES = data_list[idx:idx+4]
                TLV_LEN = int(TLV_LEN_BYTES[0], 16) + (int(TLV_LEN_BYTES[1], 16) << 8) + \
                          (int(TLV_LEN_BYTES[2], 16) << 16) + (int(TLV_LEN_BYTES[3], 16) << 24)
                idx += 4

                NUM_OF_DET_BYTES = data_list[idx:idx+2]
                NUM_OF_DET = int(NUM_OF_DET_BYTES[0], 16) + (int(NUM_OF_DET_BYTES[1], 16) << 8)
                idx += 2

                XYZQFORMAT_BYTES = data_list[idx:idx+2]
                XYZQFORMAT = int(XYZQFORMAT_BYTES[0], 16) + (int(XYZQFORMAT_BYTES[1], 16) << 8)
                idx += 2

                for i in range(NUM_OF_DET):
                    DOPPLER_BYTES = data_list[idx:idx+2]
                    DOPPLER = int(DOPPLER_BYTES[0], 16) + (int(DOPPLER_BYTES[1], 16) << 8)
                    if DOPPLER > 32767:
                        DOPPLER -= 65536
                    DOPPLER /= pow(2, XYZQFORMAT)
                    idx += 2

                    PEAK_VAL_BYTES = data_list[idx:idx+2]
                    PEAK_VAL = int(PEAK_VAL_BYTES[0], 16) + (int(PEAK_VAL_BYTES[1], 16) << 8)
                    idx += 2

                    X_BYTES = data_list[idx:idx+2]
                    x = int(X_BYTES[0], 16) + (int(X_BYTES[1], 16) << 8)
                    if x > 32767:
                        x -= 65536
                    x /= pow(2, XYZQFORMAT)
                    idx += 2

                    Y_BYTES = data_list[idx:idx+2]
                    y = int(Y_BYTES[0], 16) + (int(Y_BYTES[1], 16) << 8)
                    if y > 32567:
                        y -= 65536
                    y /= pow(2, XYZQFORMAT)
                    idx += 2

                    idx += 6

                    Detect_Points.append({'D_X': x, 'D_Y': y,'Doppler':DOPPLER})

        elif NUM_OF_TLV == 2:
            for i in range(NUM_OF_TLV):
                TLV_TYPE_BYTES = data_list[idx:idx+4]
                TLV_TYPE = int(TLV_TYPE_BYTES[0], 16) + (int(TLV_TYPE_BYTES[1], 16) << 8) + \
                           (int(TLV_TYPE_BYTES[2], 16) << 16) + (int(TLV_TYPE_BYTES[3], 16) << 24)
                idx += 4

                if TLV_TYPE == 1:
                    TLV_LEN_BYTES = data_list[idx:idx+4]
                    TLV_LEN = int(TLV_LEN_BYTES[0], 16) + (int(TLV_LEN_BYTES[1], 16) << 8) + \
                              (int(TLV_LEN_BYTES[2], 16) << 16) + (int(TLV_LEN_BYTES[3], 16) << 24)
                    idx += 4

                    NUM_OF_DET_BYTES = data_list[idx:idx+2]
                    NUM_OF_DET = int(NUM_OF_DET_BYTES[0], 16) + (int(NUM_OF_DET_BYTES[1], 16) << 8)
                    idx += 2

                    XYZQFORMAT_BYTES = data_list[idx:idx+2]
                    XYZQFORMAT = int(XYZQFORMAT_BYTES[0], 16) + (int(XYZQFORMAT_BYTES[1], 16) << 8)
                    idx += 2

                    for i in range(NUM_OF_DET):
                        DOPPLER_BYTES = data_list[idx:idx+2]
                        DOPPLER = int(DOPPLER_BYTES[0], 16) + (int(DOPPLER_BYTES[1], 16) << 8)
                        if DOPPLER > 32767:
                            DOPPLER -= 65536
                        DOPPLER /= pow(2, XYZQFORMAT)
                        idx += 2

                        PEAK_VAL_BYTES = data_list[idx:idx+2]
                        PEAK_VAL = int(PEAK_VAL_BYTES[0], 16) + (int(PEAK_VAL_BYTES[1], 16) << 8)
                        idx += 2

                        X_BYTES = data_list[idx:idx+2]
                        x = int(X_BYTES[0], 16) + (int(X_BYTES[1], 16) << 8)
                        if x > 32767:
                            x -= 65536
                        x /= pow(2, XYZQFORMAT)
                        idx += 2

                        Y_BYTES = data_list[idx:idx+2]
                        y = int(Y_BYTES[0], 16) + (int(Y_BYTES[1], 16) << 8)
                        if y > 32567:
                            y -= 65536
                        y /= pow(2, XYZQFORMAT)
                        idx += 2

                        idx += 6

                        Detect_Points.append({'D_X': x, 'D_Y': y,'Doppler':DOPPLER})

                elif TLV_TYPE == 3:
                        TLV_LEN_BYTES = data_list[idx:idx+4]
                        TLV_LEN = int(TLV_LEN_BYTES[0], 16) + (int(TLV_LEN_BYTES[1], 16) << 8) + \
                                  (int(TLV_LEN_BYTES[2], 16) << 16) + (int(TLV_LEN_BYTES[3], 16) << 24)
                        idx += 4
    
                        NUM_OF_TRACK_BYTES = data_list[idx:idx+2]
                        NUM_OF_TRACK = int(NUM_OF_TRACK_BYTES[0], 16) + (int(NUM_OF_TRACK_BYTES[1], 16) << 8)
                        idx += 2
    
                        XYZQFORMAT_BYTES = data_list[idx:idx+2]
                        XYZQFORMAT = int(XYZQFORMAT_BYTES[0], 16) + (int(XYZQFORMAT_BYTES[1], 16) << 8)
                        idx += 2
    
                        #idx += 2 #Tracking ID
                        try:
                        
                            for i in range(NUM_OF_TRACK):
                                TRACK_X_BYTES = data_list[idx:idx+2]
                                TRACK_X = int(TRACK_X_BYTES[0], 16) + (int(TRACK_X_BYTES[1], 16) << 8)
                                if TRACK_X > 32767:
                                    TRACK_X -= 65536
                                TRACK_X /= pow(2, XYZQFORMAT)
                                idx += 2
        
                                TRACK_Y_BYTES = data_list[idx:idx+2]
                                TRACK_Y = int(TRACK_Y_BYTES[0], 16) + (int(TRACK_Y_BYTES[1], 16) << 8)
                                if TRACK_Y > 32567:
                                    TRACK_Y -= 65536
                                TRACK_Y /= pow(2, XYZQFORMAT)
        
                                if(TRACK_Y < 0.8):
                                    idx = idx + 10
                                    break
        
                                idx += 2
        
                                TRACK_VX_BYTES = data_list[idx:idx+2]
                                TRACK_VX = int(TRACK_VX_BYTES[0], 16) + (int(TRACK_VX_BYTES[1], 16) << 8)
                                if TRACK_VX > 32767:
                                    TRACK_VX -= 65536
                                TRACK_VX /= pow(2, XYZQFORMAT)
                                idx += 2
        
                                TRACK_VY_BYTES = data_list[idx:idx+2]
                                TRACK_VY = int(TRACK_VY_BYTES[0], 16) + (int(TRACK_VY_BYTES[1], 16) << 8)
                                if TRACK_VY > 32567:
                                    TRACK_VY -= 65536
                                TRACK_VY /= pow(2, XYZQFORMAT)
                                idx += 2
        
                                idx += 4
        
                                Track_Objects.append({'T_X': TRACK_X, 'T_Y': TRACK_Y, 'T_Vx':TRACK_VX, 'T_Vy':TRACK_VY})
                        except Exception as e:
                            return None
        #print(Detect_Points)
        #print(Track_Objects)
        return sorted(Detect_Points, key=lambda x: x['D_Y']),sorted(Track_Objects, key=lambda x: x['T_Y'])

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

def main():
    """
    Main function - replaces your original CAN logging loop
    """
    print("=== CAN Frame Processor ===")
    print("High-performance CAN message collection with frame-based processing\n")

    # Configuration - UPDATE THESE VALUES
    CHANNEL = '0'  # Your CAN channel
    BITRATE = 500000  # Your CAN bitrate
    csv_filename = datetime.now()
    csv_filename=csv_filename.strftime('%H_%M_%m_%d_%Y.csv')
    CSV_FILE = csv_filename

    # Create processor with your configuration
    processor = CANFrameProcessor(
        channel=CHANNEL,
        bitrate=BITRATE,
        csv_filename=CSV_FILE
    )

    try:
        # Start processing
        if not processor.start_processing():
            print("❌ Failed to start CAN processing")
            return

        print("✅ CAN frame processing is running...")
        print("📊 Statistics will be displayed every 10 seconds")
        print("🛑 Press Ctrl+C to stop\n")

        # Monitor and display statistics
        while True:
            time.sleep(10.0)  # Print stats every 10 seconds
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
