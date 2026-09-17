# OpenAMS UI API v1

OpenAMS exposes a small, versioned status and command contract for native user
interfaces. It is intentionally independent of AFC, a particular OpenAMS
controller family, and the manager's internal state-machine implementation.

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
extruder reload, sensor checks, and nozzle cleaning. The manager rejects an
unknown slot or a slot that is not a member of `GROUP` before starting the
load.

### Unload

```gcode
OPENAMS_UNLOAD
```

This runs the complete configured cutter and extruder unload sequence.

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
must ignore unknown fields. A client may display all lanes, but it must not
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
