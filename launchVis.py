# -*- coding: utf-8 -*-
"""
Created on Fri Feb 14 12:05:16 2025

@author: User
"""

import subprocess
import time

def start_app():
    # Start parserSerial.py first (server)
    print("Starting parserSerial.py (server)...")
    server_process = subprocess.Popen(["python", "CAN_Frame_Processor_v1.4.py"])

    # Wait for 500ms before starting the client
    time.sleep(0.5)

    # Start plotVisClient.py (client)
    print("Starting plotVisClient.py (client)...")
    client_process = subprocess.Popen(["python", "Orion_Radar_Visualizer_v1.4.py"])

    # Keep the main process alive
    server_process.wait()
    client_process.wait()

if __name__ == "__main__":
    start_app()
