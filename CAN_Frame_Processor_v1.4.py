# -*- coding: utf-8 -*-
"""
Entry point kept for compatibility: the implementation now lives in
can_frame_processor.py.

The code had to move to a module name Python can import — a dot in a file name
makes it unreachable by `import`, which blocks both unit testing and freezing
the app into an executable. Running this script still works exactly as before:

    python CAN_Frame_Processor_v1.4.py --source replay:sample_log.csv
"""

from can_frame_processor import main

if __name__ == "__main__":
    main()
