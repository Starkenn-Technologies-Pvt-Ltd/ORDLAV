# -*- coding: utf-8 -*-
"""
Entry point kept for compatibility: the implementation now lives in
orion_radar_visualizer.py.

See the note in CAN_Frame_Processor_v1.4.py for why the code moved. Running
this script still works exactly as before:

    python Orion_Radar_Visualizer_v1.4.py --theme light
"""

from orion_radar_visualizer import main

if __name__ == "__main__":
    main()
