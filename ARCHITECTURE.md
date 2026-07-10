# OpenAMS Klipper plugin — architecture

This document is the durable reference for how the host plugin is structured,
how it talks to the OpenAMS mainboard firmware, and how to validate it. It is
meant to stay accurate as the code evolves; update it with behavioural changes.

## 1. Overview

The plugin drives one or more OpenAMS mainboards over the Klipper MCU protocol
(CAN). Each board ("OAMS unit") has four bays; filament is fed through a hub and
a shared **FPS** (Filament Buffer Pressure Sensor) into one toolhead extruder.
The plugin decides *what* to load/unload, handles filament runout with
automatic spool reload, and exposes G-code commands and webhooks.

Design goal: keep all decision logic in **pure, Klipper-free modules** that are
exhaustively unit-testable, and confine Klipper/MCU/reactor side effects to thin
adapters. There is no supported "managerless" mode — `[oams_manager]` is the
single coordinator and validator.

## 2. Module map

| File | Purity | Responsibility |
|------|--------|----------------|
| `oams_state.py`    | **pure** | Immutable per-lane state machine (the *store*): load/unload/calibrate lifecycle, runout sub-machine, op generations, deadlines. `reduce(system, action, world, now) -> (system, [effects])`. |
| `oams_topology.py` | **pure** | Immutable config model: FPS lanes ↔ OAMS units ↔ filament groups, with all cross-section validation and the runtime group-edit operations. |
| `oams_runtime.py`  | reactor  | The only place that executes effects: arms deadline timers, settles G-code waiters, runs the (legacy) stall watchdog. Holds the live `SystemState`. |
| `oams.py`          | driver   | One per `[oams]`: sends firmware commands, mirrors telemetry, negotiates the protocol, turns firmware replies into `OpCompleted` actions. |
| `follower.py`      | driver   | One per `[follower]`: an inline stepper as a SINGLE-BAY unit, closed-loop controlled by custom Klipper MCU firmware (see `FOLLOWER_PROTOCOL.md`). Hosts the FPS-forwarding and motion-queue feed-forward streams; PRE switch = "ready", POST switch = "loaded". |
| `oams_manager.py`  | adapter  | Builds + validates the topology, owns the runtime, registers `OAMSM_*` commands and webhooks, drives the monitor tick, hosts runtime group editing. |
| `fps.py`           | driver   | One feed-path pressure sensor; reads the FPS ADC (mainline or Kalico API, auto-detected). |
| `filament_group.py`| config   | Thin holder: parses its own bay list only. All cross-validation is the manager's. |
| `hdc1080.py`, `external_irq.py` | driver | Ancillary sensors. |

## 3. The pure-core pattern

`oams_state` and `oams_topology` import nothing from Klipper and do no I/O. They
take plain data in and return plain data (or new immutable state) out. This
makes the orchestration rules testable without a printer (`test_oams_state.py`,
`test_oams_topology.py` run under plain `unittest`), and lets the runtime-edit
API reuse the *exact same* validation the config loader uses.

The adapters (`oams_runtime`, `oams_manager`, `oams.py`) translate between
Klipper objects and the pure cores, and perform the side effects the cores
request as data (effects like `StartLoad`, `ArmDeadline`, `Settle`).

## 4. Command lifecycle

A blocking G-code command (e.g. `OAMSM_LOAD_FILAMENT`):

1. `oams_manager.cmd_LOAD_FILAMENT` → `runtime.request(fps, Load(...))`.
2. `Runtime.request` registers a `ReactorCompletion` and dispatches the action.
3. `reduce()` returns new state + effects; the runtime executes them — here
   `StartLoad` (driver sends the firmware command) and `ArmDeadline`.
4. The G-code handler blocks on `completion.wait()`. G-code is sequential, so it
   *must* block until the firmware op completes — this is inherently host-side.
5. The firmware later sends `oams_action_status(2)`. The driver marshals it onto
   the reactor thread and dispatches `OpCompleted`. `reduce()` produces a
   `Settle` effect; the runtime completes the `ReactorCompletion`; `wait()`
   returns and the handler responds.

All dispatch happens on the reactor thread (serial-thread replies are marshalled
via `register_async_callback`), so the store needs no locks.

## 5. Firmware protocol contract

Two firmware protocols exist: the OAMS mainboard protocol below, and the
inline-follower protocol, which is fully specified in `FOLLOWER_PROTOCOL.md`
(same op-code enum values, gen echo, one-terminal-status and firmware-owned
liveness — mandatory from its v1, integer-only wire units).

The firmware is the single runtime source of truth for the protocol and
publishes it into the Klipper data dictionary; the host reads it at connect via
`mcu.get_constants()`. Everything is optional with built-in fallbacks, so the
plugin runs against old firmware that publishes nothing ("legacy mode").

Published integer constants (namespaced `OAMS_*`):
- `OAMS_PROTOCOL_VERSION` — contract version; bumped on any protocol change.
- action enum `OAMS_STATUS_*`: LOADING=0, UNLOADING=1, FORWARD_FOLLOWING=2,
  REVERSE_FOLLOWING=3, COASTING=4, STOPPED=5, CALIBRATING=6, ERROR=7.
- code enum `OAMS_OP_CODE_*`: SUCCESS=0, ERROR_UNSPECIFIED=1, ERROR_BUSY=2,
  SPOOL_ALREADY_IN_BAY=3, NO_SPOOL_IN_BAY=4, ERROR_KLIPPER_CALL=5,
  CANCEL_LOAD_SPOOL=6, TIMEOUT=7.
- follower dir: `OAMS_FOLLOWER_REVERSE`=0, `OAMS_FOLLOWER_FORWARD`=1.

How the host treats them (`oams.py:_resolve_protocol`):
- The **action enum** is read dynamically (it is interpreted only in the
  driver).
- The **op-code / follower-direction enums** flow into the pure reducer, which
  cannot read the dictionary, so they are **validated** against the host's
  compiled-in constants and a mismatch is logged loudly (the version gate is the
  real guard).

Semantic guarantees the host relies on:
- **Exactly one completion-class status per op**, in op order. Completion-class
  = action ∈ {LOADING, UNLOADING, CALIBRATING, ERROR} or code == 5.
- `ERROR_BUSY(2)` is a *rejection* carrying the requested action (an op was
  already in flight). The host keeps a single in-flight op per lane.
- `ERROR_KLIPPER_CALL(5)` only co-occurs with CALIBRATING; `COASTING(4)` is never
  emitted; follower statuses (2,3,5) are non-terminal replies to the follower
  command and are dropped.
- After a load SUCCESS the firmware auto-starts the forward follower.
- A PTFE-calibration completion returns the measured length (encoder clicks) in
  `value`; `ptfe_length` is in encoder clicks, `0` = uncalibrated.

Version-gated host behaviour:
- **v1+**: read/validate the published enums (above).
- **v2+** (generation matching): the firmware exposes `oams_cmd_*2 … gen=%c` and
  replies `oams_action_status2 … gen=%c`. The driver feature-detects the `*2`
  commands, sends the op generation on the wire, and matches each completion by
  the echoed gen — eliminating the FIFO heuristic used as the legacy fallback.
  `op_gen` is kept in 0..255 to fit the one-byte wire field; the reducer rejects
  any completion whose gen ≠ the in-flight op's.
- **v3+** (firmware owns liveness): the firmware runs its own no-progress
  watchdog, stops the motors on a stall, and completes the op with
  `TIMEOUT(7)`. The host then (a) treats `TIMEOUT` as an ordinary op failure
  *without* sending a cancel (hardware already stopped), (b) downgrades its
  authoritative 120 s op deadline to a coarse disconnect backstop
  (`OAMS_DISCONNECT_BACKSTOP`, 300 s, only to release a blocked G-code wait if
  the MCU dies), and (c) disables its own redundant stall watchdog. This is
  gated by `Runtime.set_firmware_liveness`, set true only when **every** unit
  reports protocol ≥ 3 (conservative: any legacy/unconnected unit keeps the full
  host deadline).

Back-compat matrix (verified by unit tests):
- new host / old fw: `*2` absent → legacy commands + FIFO; no version → legacy
  mode + built-in enum defaults; host keeps its 120 s deadline + stall watchdog.
- new host / v3 fw: `*2` + gen matching; firmware-owned liveness; `TIMEOUT`
  handled as a failure.

## 6. Configuration & validation

`[oams_manager]` validates the whole configuration in `oams_topology`, which
enforces: ≥1 FPS lane with unique names (F1); unique OAMS names and `oams_idx`
(O1) each resolving to a known lane — explicit `fps:` or the sole lane (O2);
every group bay references a defined OAMS and a bay 0–3 (G1); a group's bays
share one FPS lane (G2); a bay belongs to at most one group (G3). Errors are
user-facing and surfaced as Klipper config errors.

**Load-order independence.** Klipper does not guarantee the order in which
config sections are instantiated. `filament_group` therefore does *not* look up
OAMS objects at load time — it only parses its own bay list. The manager
force-loads every `fps` / `oams` / `filament_group` section up front
(`get_prefix_sections` + idempotent `load_object`) and validates centrally, so a
valid config never fails because of section ordering.

## 7. Runtime group editing & persistence

For the management UI, groups can be created and bays reassigned at runtime:
`OAMSM_CREATE_GROUP`, `OAMSM_DELETE_GROUP`, `OAMSM_ASSIGN_BAY`,
`OAMSM_UNASSIGN_BAY`, plus the read-only `openams/topology` webhook. Each edit:

1. goes through the same pure `oams_topology` validators (so a runtime edit can
   never produce a config the loader would reject);
2. **persists first**: the affected `[filament_group …]` sections are rewritten
   in place in the OpenAMS config file (`oams_config_io.apply_group_edits`,
   atomic temp-file + `os.replace`), preserving every other section, option and
   comment;
3. then swaps the live model and rebuilds the derived runtime maps
   (`_rebuild_derived`).

Persisting before swapping means a file-write failure aborts the edit with the
model untouched — saved and live state never diverge. The change takes effect
immediately *and* survives a restart with **no `SAVE_CONFIG`**: Klipper's
`SAVE_CONFIG` can only rewrite the main `printer.cfg`, never an included subfile
like `oams.cfg`, so the plugin edits its own file directly. The target file is
`[oams_manager] openams_config_path` (default: `oams.cfg` next to the main
printer config). Reassigning a bay **moves** it (never silently shares);
`OAMSM_DELETE_GROUP` removes the section outright. Edits are refused on a lane
that is mid-op, handling a runout, or printing from the affected group.

The same in-place writeback (via `oams_config_io.set_option` and the manager's
`persist_config_option`) is used for **calibration** results
(`OAMS_CALIBRATE_PTFE_LENGTH`, `OAMS_CALIBRATE_HUB_HES`), which write
`ptfe_length` / `hub_hes_on` back onto the `[oams …]` section — again because
`SAVE_CONFIG` cannot reach them in `oams.cfg`.

## 8. Tunables

Config-exposed (all defaulted, so none are required in the config):
- `[oams_manager] reload_before_toolhead_distance` — extra margin before the
  runout reload point.
- `[oams_manager] monitor_interval` — runout/health monitor period (default 1 s).
- `[oams_manager] openams_config_path` — file that runtime writeback (group
  edits, calibration results) is written to (default: `oams.cfg` next to the
  main printer config).
- `[fps] extruder`, `[oams] fps:` — topology wiring.

Intentionally internal (safety-relevant timings, kept out of config to avoid
foot-guns; change in `oams_state.py`/`oams_runtime.py` with care): the op
deadline / disconnect backstop, `PAUSE_DISTANCE` (runout geometry; note
`LaneWorld.path_len` is always MILLIMETRES — the OAMS driver converts its
encoder clicks via `FILAMENT_PATH_LENGTH_FACTOR` in `oams.py`, the follower
uses its `path_length` directly), and the host stall-detection thresholds
(only used against
pre-v3 firmware).

## 9. Validation

**Unit tests** (no Klipper, no hardware, no third-party deps):
```
python3 -m unittest discover -s test
```
CI runs these on every push/PR (`.github/workflows/tests.yml`). Coverage: the
pure reducer (`test_oams_state`), the runtime effect executor with a fake
reactor (`test_oams_runtime`), the driver's protocol negotiation with a stubbed
`mcu` (`test_oams_driver`), the pure topology (`test_oams_topology`), and the
manager's validation + group editing with a fake printer (`test_oams_manager`).

**Hardware bring-up — read-only.** `OAMSM_SELFTEST` reports, without moving
filament: each FPS reading and extruder binding; every OAMS unit's connection,
negotiated protocol version, gen-matching and liveness flags, and per-bay
ready/loaded sensors; the validated topology; and live lane state — ending in
`PASS` or `WARN -> …`. Run it first on any new setup or after a firmware update.

**Hardware integration — manual, destructive (moves filament).** With a spool in
a ready bay, verify each capability surfaces exactly one completion:

1. `OAMSM_SELFTEST` → `PASS`; confirm the target bay shows `ready`.
2. `OAMSM_LOAD_FILAMENT GROUP=<g>` → "loaded"; `current_spool` set; forward
   follower running.
3. `OAMSM_FOLLOWER ENABLE=1 DIRECTION=1` / `ENABLE=0` → follower starts/stops;
   `following` reflects it in `openams/status`.
4. `OAMSM_UNLOAD_FILAMENT` → "unloaded"; bay no longer loaded; follower stopped.
5. `OAMS_CALIBRATE_PTFE_LENGTH SPOOL=<n>` → returns a length in clicks; written
   straight into `oams.cfg` (no `SAVE_CONFIG`).
6. Runout (v3 firmware): start a print from a group with a spare ready bay; cut
   the active filament. Expect PAUSING → COASTING → auto-reload of the spare, or
   a PAUSE if no spare. The lane never wedges.
7. Stall (v3 firmware): block a bay so the encoder cannot advance during a load.
   Expect the firmware to stop and complete with `TIMEOUT`, surfaced as a load
   failure — no host-side cancel, no double completion.
8. Runtime edit: `OAMSM_CREATE_GROUP GROUP=test`,
   `OAMSM_ASSIGN_BAY GROUP=test OAMS=<o> BAY=<b>`, check `openams/topology`,
   confirm `oams.cfg` was updated in place (no `SAVE_CONFIG` needed) and the
   edit survives a restart.
