# -*- coding: utf-8 -*-
"""
Launch the ORDLAV pipeline: the CAN frame processor (socket server) plus the
radar visualizer (socket client).

Any extra arguments are forwarded to CAN_Frame_Processor_v1.4.py, so the same
launcher drives hardware or a replayed log:

    python launchVis.py                                   # default source
    python launchVis.py --source replay:sample_log.csv --no-raw-log --loop
"""

import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PROCESSOR = os.path.join(HERE, "CAN_Frame_Processor_v1.4.py")
VISUALIZER = os.path.join(HERE, "Orion_Radar_Visualizer_v1.4.py")

# Give the processor time to bind localhost:5001 before the client connects.
# (The client retries every 3 s anyway, so this only avoids a slow first frame.)
SERVER_STARTUP_DELAY_S = 0.5


def start_app(processor_args):
    python = sys.executable  # stay inside the active virtualenv

    print("Starting CAN frame processor (server)...")
    server_process = subprocess.Popen([python, PROCESSOR] + processor_args,
                                      cwd=HERE)

    time.sleep(SERVER_STARTUP_DELAY_S)

    print("Starting radar visualizer (client)...")
    client_process = subprocess.Popen([python, VISUALIZER], cwd=HERE)

    try:
        # Whichever side exits first, shut the other down so no orphan is left
        # holding port 5001.
        while True:
            if server_process.poll() is not None:
                client_process.terminate()
                break
            if client_process.poll() is not None:
                server_process.terminate()
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nShutting down...")
        server_process.terminate()
        client_process.terminate()

    for process in (server_process, client_process):
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()


if __name__ == "__main__":
    start_app(sys.argv[1:])
