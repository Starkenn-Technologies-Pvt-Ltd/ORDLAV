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

Always install into a virtualenv. Besides the usual isolation, it sidesteps a
trap specific to this project: PyPI hosts an unrelated package named `can`
alongside `python-can`, and if the wrong one is installed globally the app
fails with `module 'can' has no attribute 'interface'`. A fresh venv cannot
inherit that.

### Windows

```powershell
cd D:\RADAR\GitHub\ORDLAV
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Calling `.\.venv\Scripts\python.exe` directly avoids `Activate.ps1`, which
PowerShell's execution policy often blocks. Verify the right CAN package:

```powershell
.\.venv\Scripts\python.exe -c "import can; print(can.__version__, can.__file__)"
```

That must print a `python-can` 4.x version and a path inside `.venv`.

### macOS / Linux

```bash
python3.12 -m venv ~/.venvs/ordlav
~/.venvs/ordlav/bin/python -m pip install -r requirements.txt
```

Two macOS-only gotchas, both of which stop the GUI with
`Could not find the Qt platform plugin "cocoa"`:

1. **Keep the virtualenv outside `~/Desktop`, `~/Documents` and `~/Downloads`.**
   Those folders are TCC-protected. A process without access to them can still
   `dlopen` a library by absolute path, but its *directory listings* come back
   empty — so Qt's plugin scan finds nothing and the visualizer dies, even
   though every plugin file is present. The project itself can live on the
   Desktop; only the virtualenv has to move. (Granting the terminal
   Full Disk Access in System Settings → Privacy & Security works too.)
2. **`PyQt5-Qt5` is pinned to 5.15.16 on macOS only.** The 5.15.19 wheel
   ships a macOS platform plugin that fails to load on Apple silicon. The pin
   carries a `sys_platform == "darwin"` marker because Windows only publishes
   5.15.2 of that package — pinning it everywhere breaks the Windows install.

In the commands below, substitute your interpreter —
`.\.venv\Scripts\python.exe` on Windows, `~/.venvs/ordlav/bin/python` on
macOS — wherever `python` appears.

## Running without the radar bench

A laptop has no Kvaser interface, so the default source cannot open. Replay a
logged run instead — generate one first if you have no bench log to hand:

```powershell
python tools\make_sample_log.py -o sample_log.csv -n 200
python launchVis.py --source replay:sample_log.csv --no-raw-log --wait-for-visualizer 15
```

Set `ORDLAV_SOURCE` to avoid repeating `--source`:

```powershell
$env:ORDLAV_SOURCE = "replay:sample_log.csv"     # PowerShell
```

`--source virtual` opens an empty in-process bus, which is useful for checking
that the GUI and socket come up without any data flowing.

## Running

The processor takes its messages from `--source`:

| Source | Use |
|---|---|
| `kvaser` | the real bench via Kvaser's own driver (Windows/Linux; default there) |
| `socketcan` | Linux in-kernel CAN. Bring the interface up first: `sudo ip link set can0 up type can bitrate 500000`, then `--source socketcan --channel can0` |
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

## Logs written

Each run writes three CSVs alongside the raw CAN log:

| File | Shape |
|---|---|
| `parsed_*.csv` | one row per frame, single object per category (legacy; unchanged) |
| `frames_*.csv` | one row per frame: ego speed, alarm, thresholds, object counts |
| `objects_*.csv` | **one row per object per frame** |

`objects_*.csv` is the one to analyse. Every source — the CAN objects, the HIL
targets, and the radar's own tracks, clusters and points — is normalised to a
single convention: **`distX` longitudinal (ahead positive), `distY` lateral
(right positive)**. The mmWave TLV stream names its axes the other way round,
so it is converted at the parser rather than leaving each consumer to remember
which source means what. The `source` column says where each row came from.

The legacy wide CSV keeps its exact shape, so the analysis script and anything
else reading it are unaffected.

## Reverse-engineering the undecoded messages

The DBC documents `0x334`–`0x338`, `0x500` and `0x6F0` as placeholders with
unknown layout, and no documented message carries an object ID, orientation,
classification or size. `tools/probe_unknown_ids.py` characterises those
messages across a run — which bits move, which bytes behave like counters,
slot indices or flags, and which 8/16-bit fields track a reference signal:

```bash
python tools/probe_unknown_ids.py bench_log.csv
```

Correlation is evidence, not proof. Confirm a candidate against a second log
with different target geometry before decoding it for real.

## Replay logs

Any raw log the processor writes (`HH_MM_mm_dd_YYYY.csv`) can be replayed
straight back. To generate a synthetic one — a target closing from 80 m to
12 m with point cloud, clusters, tracks and HIL ground truth:

```bash
~/.venvs/ordlav/bin/python tools/make_sample_log.py -o sample_log.csv -n 200
```

## What the radar sends

Decoded per DBC V6:

| ID | Message | Notes |
|---|---|---|
| `0x510/511/512` | moving objects 1-3 | slots ranked by proximity, closest first |
| `0x590/591/592` | stationary objects 1-3 | same |
| `0x501` / `0x581` | object counts | plus cycle time and a cycle counter |
| `0x6F0` | TTC / PET / relevant object | |
| `0x6F1` | alarm thresholds | 0.5 m per LSB |
| `0x001` | alarm level | 2 bits |
| `0x3F5` | ego speed | **m/s**, not km/h |
| `0x3F7` | ego yaw rate | sign plus magnitude |
| `0x100`-`0x103` | HIL targets 1-4 | with class, trajectory and RCS |

Two things worth knowing:

**The object slots are not identities.** `_1` is whatever is closest right now,
so an object moves between slots as the ranking changes. Identity has to be
established by association — see below.

**Classification is ground truth, not radar output.** Target type
(pedestrian / cycle / car / truck), trajectory and RCS come from the HIL rig on
`0x100`-`0x103`. The radar reports no class, orientation, size or acceleration
in any documented message.

## Object identity and continuity

No documented message carries an object ID, so `object_tracker.py` assigns one
during capture by associating each frame's tracks with the previous frame's.
The id appears on each track in the view and in the `assoc_id` column of
`objects_*.csv`.

Read the distinction carefully: this measures the **radar's detection
stability** — whether one physical target stays one continuously reported
object — which is what a split or a dropout actually is. It does **not** audit
the sensor's own ID handling, because the sensor exposes no ID to audit.

```bash
python tools/track_continuity.py objects_*.csv
```

reports, per identity, its lifetime, dropouts and range covered, then flags:

* **SPLIT** — two identities coexisting within a few metres, i.e. one target
  reported as two
* **CHURN** — identities that lasted only a frame or two
* **dropouts** — gaps inside an identity's lifetime

## Validation against ground truth

```bash
python Analysis/ncap_report.py objects_*.csv --scenario CCRm --source track
```

Produces, per run, each kinematic channel plotted against ground truth with the
error beneath it, plus detection metrics (detection rate, first-detection
range, longest dropout) and per-channel error statistics.

**On pass/fail.** The Euro NCAP AEB protocols (Car-to-Car v4.3.1, VRU v4.5.1)
grade *system-level* behaviour — whether the vehicle avoids the collision —
not sensor accuracy. They cannot supply the tolerance corridor, which has to
come from the radar requirement specification. Without `--tolerances` the
report measures the error and makes no judgement; it prints observed error
percentiles, which are the right input for *writing* a requirement. A
requirement derived from one run and then used to grade that same run proves
nothing. Once limits exist, copy `Analysis/tolerances.example.json` and pass it
with `--tolerances`.

What the protocols do give you is the scenario set; tag each run with
`--scenario` (CCRs, CCRm, CCRb, CPNA, CPNC, CPFA, CPLA, CBNA, CBLA) so results
are comparable across the matrix.

## Analysis

Compares radar objects (0x510 / 0x590) against HIL target 1 (0x100) and writes
per-run stats and plots. It globs `*.csv` in the **current working directory**
and **moves** each input into a folder beside it, so run it from a directory
holding only the parsed CSVs you want processed:

```bash
cd Analysis && ../~/.venvs/ordlav/bin/python plot_and_analyse_data.py
```
