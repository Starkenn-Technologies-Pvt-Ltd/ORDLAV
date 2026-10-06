# -*- coding: utf-8 -*-
"""
Single entry point for the packaged ORDLAV application.

A frozen executable cannot start the two halves by re-running .py files, so
this module can host both in one process: the frame processor runs on its own
daemon threads while the Qt visualizer owns the main thread, exactly as they
do when started separately. They still talk over localhost:5001, so nothing
about the protocol or either half changes.

    ORDLAV.exe                                   both halves (default)
    ORDLAV.exe --source replay:bench_log.csv     replay a logged run
    ORDLAV.exe --mode processor                  headless capture only
    ORDLAV.exe --mode visualizer                 attach to a processor elsewhere
"""

import argparse
import sys
import time

import can_source


def build_parser():
    parser = argparse.ArgumentParser(
        prog="ORDLAV",
        description="Orion radar data logger, analyzer and visualizer.")
    parser.add_argument("--mode", choices=("combined", "processor", "visualizer"),
                        default="combined",
                        help="Which halves to run (default: combined)")

    source = parser.add_argument_group("CAN source")
    source.add_argument("--source", default=None,
                        help="kvaser | virtual | replay:<log.csv> | any "
                             "python-can interface (default: kvaser on "
                             "Windows/Linux, virtual elsewhere)")
    source.add_argument("--channel", default="0", help="CAN channel (default: 0)")
    source.add_argument("--bitrate", type=int, default=500000,
                        help="CAN bitrate (default: 500000)")
    source.add_argument("--speed", type=float, default=1.0,
                        help="Replay speed multiplier; 0 = as fast as possible")
    source.add_argument("--loop", action="store_true",
                        help="Restart a replayed log when it ends")
    source.add_argument("--no-raw-log", action="store_true",
                        help="Skip the raw CAN CSV (useful when replaying one)")
    source.add_argument("--wait-for-visualizer", type=float, default=0.0,
                        metavar="SECONDS",
                        help="Hold the source until the visualizer connects, "
                             "up to SECONDS (useful when replaying a log)")

    view = parser.add_argument_group("View")
    view.add_argument("--theme", choices=("dark", "light"), default=None,
                      help="Start in this appearance (default: last used)")
    return parser


def run_processor(args, block=True):
    """Start the capture side. Returns the processor, or None if it failed."""
    from can_frame_processor import CANFrameProcessor

    processor = CANFrameProcessor(
        channel=args.channel,
        bitrate=args.bitrate,
        source=args.source,
        replay_speed=args.speed,
        replay_loop=args.loop,
        raw_log=not args.no_raw_log,
        wait_for_visualizer=args.wait_for_visualizer,
    )

    if not processor.start_processing():
        print("Failed to start CAN processing")
        return None

    if block:
        try:
            while not processor.source_exhausted.wait(timeout=1.0):
                pass
            print("End of replayed log reached")
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            processor.stop_processing()
    return processor


def run_visualizer(args, processor=None):
    """Run the Qt application on the main thread; returns its exit code."""
    from PyQt5.QtWidgets import QApplication
    import orion_radar_visualizer as viz

    app = QApplication(sys.argv[:1])
    app.setStyle('Fusion')

    window = viz.RadarVisualizerMainWindow(theme=args.theme)
    window.show()
    app.radar_interface = viz.RadarDataInterface(window)

    try:
        return app.exec_()
    finally:
        if processor is not None:
            processor.stop_processing()


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.mode == "processor":
        return 0 if run_processor(args, block=True) is not None else 1

    if args.mode == "visualizer":
        return run_visualizer(args)

    # Combined: capture in the background, GUI on the main thread.
    processor = run_processor(args, block=False)
    if processor is None:
        return 1
    # Give the socket server a moment to bind before the GUI dials in.
    time.sleep(0.2)
    return run_visualizer(args, processor=processor)


if __name__ == "__main__":
    sys.exit(main())
