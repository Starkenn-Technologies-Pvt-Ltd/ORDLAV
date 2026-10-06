# ORDLAV — Orion Radar Data Logger, Analyzer & Visualizer

Captures CAN traffic from the radar bench, reconstructs TI mmWave frames,
visualizes them live, and compares radar output against HIL ground truth.

```
CAN source ──▶ CAN_Frame_Processor_v1.4.py ──localhost:5001──▶ Orion_Radar_Visualizer_v1.4.py
(hardware,          │  raw CSV + parsed CSV                        (PyQt5 live plot)
 virtual,           ▼
 or a logged CSV)   Analysis/plot_and_analyse_data.py  ──▶ error stats + plots
```

## Setup

```bash
python3.12 -m venv ~/.venvs/ordlav
~/.venvs/ordlav/bin/python -m pip install -r requirements.txt
```

Two macOS gotchas, both of which stop the GUI with
`Could not find the Qt platform plugin "cocoa"`:

1. **Keep the virtualenv outside `~/Desktop`, `~/Documents` and `~/Downloads`.**
   Those folders are TCC-protected. A process without access to them can still
   `dlopen` a library by absolute path, but its *directory listings* come back
   empty — so Qt's plugin scan finds nothing and the visualizer dies, even
   though every plugin file is present. The project itself can live on the
   Desktop; only the virtualenv has to move. (Granting the terminal
   Full Disk Access in System Settings → Privacy & Security works too.)
2. **`PyQt5-Qt5` is pinned to 5.15.16.** The 5.15.19 wheel ships a macOS
   platform plugin that fails to load on Apple silicon.

Substitute `~/.venvs/ordlav/bin/python` for `python` in the commands below.

## Running

The processor takes its messages from `--source`:

| Source | Use |
|---|---|
| `kvaser` | the real bench (Windows/Linux; default there) |
| `virtual` | python-can in-process bus, no hardware (default on macOS) |
| `replay:<file.csv>` | re-play a raw CAN log |
| any python-can name | `socketcan`, `pcan`, `slcan`, … |

Both halves at once:

```bash
~/.venvs/ordlav/bin/python launchVis.py --source replay:sample_log.csv --no-raw-log --wait-for-visualizer 15
```

Or separately — processor first, then the visualizer:

```bash
~/.venvs/ordlav/bin/python CAN_Frame_Processor_v1.4.py --source replay:sample_log.csv --no-raw-log
```

```bash
~/.venvs/ordlav/bin/python Orion_Radar_Visualizer_v1.4.py
```

Useful flags: `--speed` (replay rate; `0` = as fast as possible), `--loop`,
`--wait-for-visualizer SECONDS` (hold a finite log until the GUI attaches),
`--no-raw-log`, `ORDLAV_DEBUG=1` for per-frame tracing.

## The view

The scene is drawn bird's-eye: range up the screen, lateral across, ego vehicle
at the origin. Detected points are coloured by Doppler (closing / static /
opening), clusters are drawn at their reported extent, tracks carry a velocity
arrow showing where they travel next, and the moving-object alarm thresholds
are shaded as L1/L2/L3 bands. Scroll the plot to zoom.

Light and dark appearance are both supported — use the toggle in the header,
or start in one explicitly:

```bash
~/.venvs/ordlav/bin/python Orion_Radar_Visualizer_v1.4.py --theme light
```

The choice is remembered between runs.

## Windows executable

`packaging/ORDLAV.spec` builds a single `ORDLAV.exe` that runs both halves in
one process — the capture threads plus the Qt view — so the bench machine needs
no Python install.

PyInstaller does not cross-compile: the `.exe` must be built on Windows. Either
let CI do it (the **Build Windows executable** workflow publishes `ORDLAV.exe`
as a build artifact on every push, and can be run on demand from the Actions
tab), or build it on the bench machine:

```bat
pip install -r requirements.txt pyinstaller
pyinstaller packaging/ORDLAV.spec --noconfirm
```

The result lands in `dist/ORDLAV.exe` and takes the same options as the scripts:

```bat
ORDLAV.exe
ORDLAV.exe --source replay:bench_log.csv --no-raw-log --wait-for-visualizer 10
ORDLAV.exe --mode processor
ORDLAV.exe --theme light
```

`--mode processor` runs headless capture only and `--mode visualizer` attaches
to a processor running elsewhere; the default runs both.

## Replay logs

Any raw log the processor writes (`HH_MM_mm_dd_YYYY.csv`) can be replayed
straight back. To generate a synthetic one — a target closing from 80 m to
12 m with point cloud, clusters, tracks and HIL ground truth:

```bash
~/.venvs/ordlav/bin/python tools/make_sample_log.py -o sample_log.csv -n 200
```

## Analysis

Compares radar objects (0x510 / 0x590) against HIL target 1 (0x100) and writes
per-run stats and plots. It globs `*.csv` in the **current working directory**
and **moves** each input into a folder beside it, so run it from a directory
holding only the parsed CSVs you want processed:

```bash
cd Analysis && ../~/.venvs/ordlav/bin/python plot_and_analyse_data.py
```
