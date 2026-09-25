# OpenAMS UI API v1

OpenAMS exposes a small, versioned status and command contract for native user
interfaces. It is intentionally independent of AFC, a particular OpenAMS
controller family, and the manager's internal state-machine implementation.

## Upgrading existing installations

An ordinary repository update followed by a Klipper restart while the printer
is idle keeps existing installations working. The API is implemented entirely
inside the already-installed `oams_manager.py`: it adds no Python dependencies,
extra-module symlinks, mandatory configuration, or MCU firmware requirements.
Rerunning the installer is **not required for this change**.

Existing `T0`/`T1`/`_TX GROUP=...` macros, automatic runout handling, and
untargeted `OAMSM_LOAD_FILAMENT` / `OAMSM_UNLOAD_FILAMENT` keep their existing
behavior, including informational replies on hardware failure. Strict error
handling is opt-in: a load with `SLOT=...`, or an unload with `STRICT=1`, raises
a G-code error on failure so the new UI sequence cannot continue after it.

To enable **HelixScreen** load/unload controls, manually merge the updated
sections from `oams_macros.cfg` into your printer's customized macro file:
`OPENAMS_LOAD`, `OPENAMS_UNLOAD`, `_TX`, `SAFE_UNLOAD_FILAMENT`, `CUT_FILAMENT`,
`_LOAD_FS_IN`, `_LOAD_FS_OUT`, `_UNLOAD_FS_OUT`, and every `_OAMS_...` helper
macro they call. Preserve your calibrated variables and printer-specific
cutter, parking, and nozzle-cleaning changes. The updated `_TX` also stops
untargeted toolchanges (`T0`...) on a failed load, unload or enabled sensor
check: it pauses and raises an error instead of continuing. Restart Klipper
after reviewing the merged macros. The installer deliberately does not replace
an existing macro file, even when rerun.

Until those macros are present, status remains available but `commands` omits
the unavailable load/unload operations. Clients must disable an operation if
its advertised command is missing, and only that operation: HelixScreen keeps
showing status and offering the advertised operations, and refuses load (and
tool changes) or unload until the matching command appears.

This is the Klipper-side support for HelixScreen, tracked in
[HelixScreen PR #1691](https://github.com/prestonbrown/helixscreen/pull/1691).
It does not add a KlipperScreen plugin or an AFC dependency.

## Discovery and subscription

`oams_manager` in `printer.objects.list` is not enough to identify this API:
a manager from before it registers the same object and publishes only
`current_group`. A client therefore queries `oams_manager` for `api_version`
and `schema` and treats OpenAMS as absent unless both are supported. It then
subscribes to the `oams_manager` object.

AFC drives OpenAMS hardware through its own objects and does not use
`oams_manager`, so a leftover `[oams_manager]` section can coexist with a
working AFC setup. A client that supports both must leave the printer to AFC
whenever the `AFC` object is present. HelixScreen does.

The status object has this shape:

```json
{
  "api_version": 1,
  "schema": "openams.manager",
  "ready": true,
  "commands": {
    "load": "OPENAMS_LOAD",
    "unload": "OPENAMS_UNLOAD",
    "cancel": "OAMSM_LOAD_FILAMENT_CANCEL",
    "reset": "OAMSM_CLEAR_ERRORS"
  },
  "lanes": [
    {
      "id": "fps",
      "state": "loaded",
      "current_group": "T0",
      "current_slot": 0,
      "following": false,
      "direction": 0,
      "message": null,
      "pressure": 0.48,
      "set_point": 0.5
    }
  ],
  "units": [
    {
      "id": "1",
      "name": "unit1",
      "kind": "oams",
      "topology": "hub",
      "lane": "fps",
      "connected": true,
      "slots": [
        {"id": 0, "bay": 0, "ready": true, "loaded": true}
      ]
    }
  ],
  "groups": [
    {"name": "T0", "lane": "fps", "slots": [0]}
  ],
  "current_group": "T0"
}
```

`current_group` remains at the top level for compatibility with existing
macros. New clients should otherwise consume only the versioned fields above.

The legacy `openams/status` webhook remains available. Its original fields are
unchanged and the v1 snapshot is available below its `api` key.

## Model

- A **lane** is one independently operated filament path/FPS. Current master
  publishes one lane named `fps`. `pressure` is the lane's FPS reading, from
  0.0 (no pressure) to 1.0 (fully compressed); the FPS measures compression
  only, never tension. It is rounded to two decimals and republished only
  once it moves by 0.02, so sensor noise does not resend the lanes on every
  poll. `set_point` is the compression the feeding unit's hub motor regulates
  to (its `fps_target`), or `null` when no unit reports one.
- A **unit** is a physical feeder attached to one lane. `kind` is a stable
  machine identifier; `topology` is the hardware-neutral rendering/operation
  shape (`hub`, `linear`, `parallel`, or `mixed`). Clients must branch on
  `topology`, not on a family name. A client that meets a `topology` value it
  does not know may draw that unit as a hub: slots are addressed by id, so
  operations are unaffected. A unit contains one or more slots.
- A **slot** has a manager-assigned integer `id` that is unique within the
  snapshot. Commands address this id; clients must not derive command
  addresses from unit names or array positions.
- A **group** names the slots eligible to satisfy a tool/material request and
  belongs to one lane.

This nesting is the forward-compatibility boundary for multi-FPS and
multi-family support. A future manager may publish several lanes and mix unit
kinds without changing the v1 meanings. Existing fields are additive within
v1; a meaning change requires a new `api_version`.

## Commands

Clients must use the command names advertised in `commands` rather than
hard-coding lower-level device commands.

### Load a selected slot

```gcode
OPENAMS_LOAD GROUP=T0 SLOT=0
```

To serve a tool or material, a client picks the slot from the group: the
member already loaded if there is one, otherwise a member whose slot reports
`ready`. HelixScreen does not send a load for a group with no ready member.

`OPENAMS_LOAD` runs the complete configured toolchange sequence: homing when
needed, safe unload/cut of the previous filament, targeted OpenAMS feed,
extruder reload, sensor checks, and nozzle cleaning. `OAMSM_VALIDATE_LOAD`
rejects a malformed, unknown, unavailable, or incorrectly grouped slot before
any homing, cutting, or unloading. The target is checked again when feeding
starts. Selecting a different slot in the same group performs a toolchange.
The new UI sequence stops on a reported hardware failure or failed enabled
toolhead sensor check. Temperatures, cutter positions, and motion distances
remain the responsibility of the printer's configured macros.

### Unload

```gcode
OPENAMS_UNLOAD
```

This runs the complete configured cutter and extruder unload sequence with
`STRICT=1`. Homing precedes extrusion, which uses relative mode. When the
manager reports no loaded slot, this command does not move the printer.

### Cancel or reset

```gcode
OAMSM_LOAD_FILAMENT_CANCEL
OAMSM_CLEAR_ERRORS
```

Cancel requests cancellation of an active hardware load. It does not promise
that already queued physical toolhead moves have stopped. Reset clears OpenAMS
errors and re-determines state.

`OAMSM_LOAD_FILAMENT_CANCEL` is ordinary G-code, so it runs only once Klipper's
G-code queue is free. It can reach a load the manager runs on its own, such as
a runout reload. It cannot interrupt a load started with the advertised load
command, because `OPENAMS_LOAD` holds the queue until the load completes or
fails; by then there is nothing left to cancel. Clients must not offer cancel
for a load they started. HelixScreen refuses it as busy and leaves its own
record of the load in place until the command returns. The Klippy-side
`openams/cancel_load` webhook can interrupt a load, but Moonraker does not
proxy it to clients.

## Compatibility rules

Clients must fail closed when `api_version` is absent or unsupported: they
treat OpenAMS as absent rather than reporting an error. They must ignore
unknown fields and check each command's availability separately. While `ready`
is false, clients show status but send no command. A client may display all
lanes, but it must not silently combine independent lanes into a single
current-slot/action value. With several lanes loaded, `OPENAMS_UNLOAD` cannot
say which lane it empties, so clients do not send it.

Current master has a single FPS and therefore one active lane. The same API is
designed for the future manager as follows:

- publish one lane entry per FPS;
- attach every unit to its owning lane;
- keep slot ids unique for the snapshot;
- attach each group to exactly one lane;
- route a targeted load by slot id through that lane's runtime;
- use the same advertised high-level commands so a UI never depends on a
  reducer class or hardware-family-specific G-code.

## Regression verification

The regular suite checks the status contract, legacy webhook fields, slot
selection, legacy failure behavior, and importing through an old installation's
individual extras symlinks without installing any new module.

The optional integration suite uses the G-code dispatcher and macro engine from
Klipper commit `57a018ad7bbacde28c35305fd79cb9488bd201db`, with fake hardware.
It compares command/motion traces and final state against OpenAMS master commit
`01de61b6ccf406b8bd17e4ff8a5e1250ec942ac1`, using both unchanged old macros and
the new macros. It also checks validation before motion, within-group slot
changes, error propagation, and sensor failures:

```bash
OPENAMS_KLIPPER_DIR=/path/to/klipper python3 -m pytest -q
```

The checkout must contain the pinned commit. Tests read its committed source
without changing that checkout or accessing a printer. These checks exercise
the real parser/template engine with fake devices; they are not a physical
printer or MCU validation.
