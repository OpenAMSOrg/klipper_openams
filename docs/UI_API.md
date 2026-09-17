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
`OPENAMS_LOAD`, `OPENAMS_UNLOAD`, `_TX`, `SAFE_UNLOAD_FILAMENT`, `_LOAD_FS_IN`,
`_LOAD_FS_OUT`, and `_UNLOAD_FS_OUT`. Preserve your calibrated variables and
printer-specific cutter, parking, and nozzle-cleaning changes. Restart Klipper
after reviewing the merged macros. The installer deliberately does not replace
an existing macro file, even when rerun.

Until those macros are present, status remains available but `commands` omits
the unavailable load/unload operations. Clients must disable an operation if
its advertised command is missing. The HelixScreen v1 adapter requires all
four commands and fails closed on an incomplete command map.

This is the Klipper-side support for HelixScreen, tracked in
[HelixScreen PR #1691](https://github.com/prestonbrown/helixscreen/pull/1691).
It does not add a KlipperScreen plugin or an AFC dependency.

## Discovery and subscription

A client discovers OpenAMS when `printer.objects.list` contains
`oams_manager`. It then subscribes to the `oams_manager` object. The status
object has this shape:

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
      "message": null
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
  publishes one lane named `fps`.
- A **unit** is a physical feeder attached to one lane. `kind` is a stable
  machine identifier; `topology` is the hardware-neutral rendering/operation
  shape (`hub`, `linear`, `parallel`, or `mixed`). Clients must branch on
  `topology`, not on a family name. A unit contains one or more slots.
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

## Compatibility rules

Clients must fail closed when `api_version` is absent or unsupported. They
must ignore unknown fields and check command availability. A client may display all lanes, but it must not
silently combine independent lanes into a single current-slot/action value.

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
