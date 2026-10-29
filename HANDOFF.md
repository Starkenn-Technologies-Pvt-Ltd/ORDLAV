# Handoff prompt — paste this into a new Claude Code session on the bench PC

---

You are picking up radar validation work for Starkenn's Orion FCWS radar. I
have both repositories cloned on this Windows PC and the radar is physically
attached via a Kvaser U100. Read this whole brief before doing anything.

## Two repositories, never mix them

| Path | Repo | Branch | Contents |
|---|---|---|---|
| `ORDLAV` | `Starkenn-Technologies-Pvt-Ltd/ORDLAV` | `new-Changes` | Python validation tooling: CAN logger, live visualizer, analysis |
| `Radar_Integrated_FCWS` | `Starkenn-Technologies-Pvt-Ltd/Radar_Integrated_FCWS` | `new-changes` | TI mmWave firmware (C) for the radar itself |

**Hard rule: tooling changes go only to ORDLAV, firmware changes only to
Radar_Integrated_FCWS. Never commit a file from one into the other.** Check
before every push.

## Safety rule for the firmware repo

This firmware drives a forward collision warning function. Do not describe any
firmware change as "flashable", "ready" or "done" unless it has been compiled
in Code Composer Studio **and** validated on the bench. You cannot validate it
by reading it. If asked for flashable code, produce a patch and say plainly
what still needs building and testing.

## What has already been established — do not re-derive this

Evidence is from the firmware source plus two bench CAN captures.

1. **The firmware source in the repo is NOT the binary flashed on the radar.**
   Three messages the source has zero live call sites for are transmitted
   thousands of times by the device: `0x6F0` (275 msgs), `0x501` (2,585),
   `0x581` (2,585). `0x510` is sent 275 times although the branch populating
   it is unreachable in the source. **Finding the real source is the highest
   priority firmware task.** Until then, firmware patches target an unknown
   baseline.

2. **Ego speed comes from J1939 CCVS on extended ID `0x18FEF111`, not
   `0x3F5`.** Both DBCs document `0x3F5`; the firmware's handler for it is
   commented out at `mss_main.c:810`. `g_f_veh_speed` is written only at
   `mss_main.c:821`. Without ego speed the radar degenerates: relative
   velocity is reported as `-ego`, every object classifies stationary, all six
   alarm thresholds stay zero, TTC divides by zero.

3. **Measured relative velocity is discarded.** `wnc_source.c:685-686`
   overwrites the tracker's velocity with `-g_f_veh_speed` and `0.0`. Velocity
   accuracy therefore cannot be validated — the transmitted value is not a
   measurement.

4. **One object maximum.** `wnc_source.c:626` breaks after the first accepted
   object; `MAX_FILTERED_COUNT` (`wnc_header.h:56`) is 1 and its clamp is dead
   code.

5. **Detection gates** at `wnc_source.c:600` and `:610`: 0.5–40 m
   longitudinal, 0.16–2.5 m lateral on one side only, so an object dead centre
   on boresight is rejected. `:620` also discards objects whose ground speed is
   within ±1 m/s.

6. **Reported range is clamped at 0.576 m.** An object held at 30 cm reported
   0.576 m, byte-identical 36 times.

7. **Alarm thresholds are hardcoded** 40/15/7 at `:702-705` and `:748-751`
   (20 m / 7.5 m / 3.5 m at the DBC's 0.5 m per LSB); the calculation is
   commented out. **TTC** is assigned at four sites with two different
   scalings and no divide-by-zero guard.

8. Two buffer overruns were found and fixed on the firmware branch: an
   unclamped track count into a 24-element array (`wnc_source.c:573`), and a
   `memset` overrunning by one element (`:954`). Those fixes are **unbuilt**.

## Tooling you already have in ORDLAV

- `python CAN_Frame_Processor_v1.4.py --list-sources` — report whether the
  radar is reachable, listen 3 s, print every CAN ID heard
- `python launchVis.py` — processor plus live visualizer
- `python tools/inject_vehicle_data.py --speed 40 --units kmh` — inject CCVS
- `python tools/decode_objects.py <raw.csv>` — decode object messages from a
  raw log, bypassing the UI
- `python tools/track_continuity.py objects_*.csv` — split / dropout / churn
- `python Analysis/ncap_report.py objects_*.csv --scenario CCRm` — accuracy vs
  ground truth
- `python tools/probe_unknown_ids.py <raw.csv>` — characterise unknown IDs
- `python -m pytest tests -q` — 29 regression tests, run before any push

Each run writes `frames_*.csv` (one row per frame) and `objects_*.csv` (one row
per object per frame, all sources normalised to distX longitudinal / distY
lateral).

## Do these now, in order

1. Confirm the radar is reachable and note which CAN IDs it sends:
   `python CAN_Frame_Processor_v1.4.py --list-sources`
   If it fails, report the exact error — do not guess. Try `--channel 1`.

2. Capture a 60 s baseline with no ego speed injected. This reproduces the
   known-bad state for comparison.

3. Capture again **with CCVS injected**, ideally a ramp:
   `python tools/inject_vehicle_data.py --ramp 0:20 --duration 60`
   Start the injector before the capture.

4. Compare the two captures and report whether these changed: alarm
   thresholds non-zero, `0x510` message count, objects classified moving, TTC
   plausible. **If nothing changes, that is itself evidence the flashed build
   differs from the source — say so rather than assuming the injection
   failed.**

5. Report findings with numbers. Do not fix firmware yet.

## The plan this feeds into

- **Track A — tooling.** Tests exist and gate CI. Next: config file for
  host/port/channel, structured logging, narrow the broad `except Exception`
  blocks, session metadata beside each log, log rotation, headless service.
- **Track B — validation campaign.** Blocked on: HIL rig connected for ground
  truth, and Starkenn defining sensor accuracy requirements. Euro NCAP grades
  vehicle-level collision avoidance, not sensor accuracy, so it cannot supply
  the tolerance corridor. Until requirements exist, reports measure error and
  decline to judge.
- **Track C — firmware decisions.** Owned by the radar team, gates everything:
  the velocity overwrite, the one-object cap, the detection gates, and finding
  the real source. These are decisions, not bugs.
- **Track D — production ADAS.** Cannot be scheduled until Track C resolves.

## How to work

Verify rather than assume — this codebase has repeatedly contradicted its own
documentation. When a finding conflicts with the DBC or a comment, trust the
captured data and the firmware source, in that order, and say which you used.
Report numbers, not impressions. If something cannot be verified from this PC,
say so instead of implying it was tested.
