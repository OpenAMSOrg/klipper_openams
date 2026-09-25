# OpenAMS for Klipper  
OpenAMS Klipper Plugin

Native UI integrations can use the versioned, AFC-independent
[OpenAMS UI API](docs/UI_API.md) exposed by `oams_manager`.
This adds the Klipper-side contract for **HelixScreen** touchscreen support
([HelixScreen draft PR #1691](https://github.com/prestonbrown/helixscreen/pull/1691)).
Existing installations retain their configuration, macros, firmware protocol,
and legacy command behavior. See [upgrading and enabling the UI](docs/UI_API.md#upgrading-existing-installations)
before enabling the new touchscreen commands. HelixScreen and KlipperScreen are
separate projects; this change integrates with HelixScreen.

## Installation  

### Automatic Installation  

Install OpenAMS using the provided script:  

```bash  
cd ~  
git clone https://github.com/OpenAMSOrg/klipper_openams.git  
cd klipper_openams  
./install-openams.sh  
```  

If your directory structure differs, you can configure the installation script with additional parameters:  

```bash  
./install-openams.sh [-k <klipper path>] [-s <klipper service name>] [-c <configuration path>] [-r <gco-routines checkout>]
```

### gco-routines dependency

The installer also clones [OpenAMSOrg/gco-routines](https://github.com/OpenAMSOrg/gco-routines)
to `~/gco-routines` and installs its package as one symlink under
`klippy/extras/gco_routines`. Use `-r` to choose a different checkout directory.
On subsequent installer runs, a clean `main` checkout is updated by fast-forward
only. No tracked Klipper source or firmware is patched, and no Python dependencies
are upgraded. Network, repository ownership, and destination checks happen before
the installer stops Klipper.

**Installation does not enable concurrent execution or change existing macros.**
The installer copies `oams_macros_ordered.cfg` into the configuration directory
when it is absent, but nothing includes it and `[gco_routines]` is not added.
The extension deliberately accepts only tested Klipper baselines and requires
Python 3.9 or newer in Klipper's environment; read its README before enabling
it. Macros that do not opt in keep stock whole-template rendering.

Existing non-Git directories, modified/diverged checkouts, other branches, and
foreign extras paths are rejected rather than overwritten. A previous manually
copied installation therefore needs an explicit, backed-up migration. The helper
does not change active `[gco_routines]` configurations: already-enabled printers
keep their selected modes. Run the installer only while the printer is idle.

OpenAMS `-u` leaves the independent gco-routines checkout, symlink and activation
configuration intact, since other macros may rely on them. To remove the extra,
first remove its activation/ordered-only configuration and then run:

```bash
python3 ~/gco-routines/tools/install_gco_routines.py --klipper ~/klipper --uninstall
```

Restart Klipper while idle after changing its configuration. Rerun the OpenAMS
installer to update gco-routines; it is not silently added to Moonraker's update
configuration.

#### Enabling concurrent toolchanges

`oams_macros.cfg` in this repository already contains the concurrent workflow.
If you keep a customized copy, merge the updated macros first (the installer
never replaces it). Then add two configuration lines and restart Klipper while
the printer is idle:

1. In `printer.cfg`, put `[gco_routines]` above `[include oams.cfg]` and
   before any other `[gcode_macro ...]` section or include that defines macros.
2. In `oams.cfg`, uncomment `[include oams_macros_ordered.cfg]`, directly after
   `[include oams_macros.cfg]`. This file only sets `render_mode: ordered` on
   `_TX`.

What changes when both are present:

- The cut, toolhead retraction and OpenAMS unload still finish first. The
  OpenAMS load (`OAMSM_LOAD_FILAMENT`) then runs in a background routine while
  `CLEAN_NOZZLE` runs, so the nozzle is cleaned during loading.
- `M400` finishes the cleaning moves, and `WAIT` joins the load before the
  load result and inlet sensor are checked and before the toolhead reload
  extrusion.
- A toolchange that starts while the printer is paused runs serially. A pause
  during the overlap follows the extension's rules (no new routine starts while
  paused); the toolchange then stops before the toolhead reload.
- Call `T0`-`T19` and `OPENAMS_LOAD` from ordinary G-code only, never inside
  another `START` block: they own their background routine.

On stock Klipper, or without the overlay, the same `oams_macros.cfg` runs every
step serially (load, inlet check, `CLEAN_NOZZLE`, reload) and never renders
`START`, `END` or `WAIT`. Stock Klipper rejects `render_mode`, so include
`oams_macros_ordered.cfg` only together with `[gco_routines]`. To turn the
feature off, comment out both lines again.

#### Changes from the previous oams_macros.cfg

These apply with or without gco-routines. If you keep a customized
`oams_macros.cfg`, merge them by hand:

- New `_OAMS_*` helper macros (`_OAMS_FINISH_UNLOAD`, `_OAMS_CONFIRM_UNLOADED`,
  `_OAMS_CONTINUE_AFTER_UNLOAD`, `_OAMS_CONTINUE_AFTER_LOAD`,
  `_OAMS_CONTINUE_AFTER_INLET`, `_OAMS_FINISH_TOOLCHANGE`,
  `_OAMS_FAIL_TOOLCHANGE`, `_OAMS_ABORT`, `_OAMS_RAISE`). Stock Klipper renders
  a whole macro before running it; each helper renders fresh, so every safety
  decision sees the result of the step before it. `_TX` and
  `SAFE_UNLOAD_FILAMENT` call them, so copy all of them.
- Failure handling: on a failed load, failed unload, enabled sensor failure, or
  a pause arriving mid-toolchange, `T0`-`T19` and `SAFE_UNLOAD_FILAMENT` turn
  the follower off, `PAUSE`, and raise a G-code error. In an SD print the print
  therefore ends through `on_error_gcode` instead of continuing. UI (`STRICT`)
  requests (`OPENAMS_LOAD`, `OPENAMS_UNLOAD`) raise without `PAUSE`.
- `CLEAN_NOZZLE` runs after the OpenAMS load and before the toolhead reload
  (previously after the reload). There is no `G0 Z15` before it, so
  `CLEAN_NOZZLE` must provide its own Z clearance.
- The toolchange is wrapped in one `SAVE_GCODE_STATE` / `RESTORE_GCODE_STATE`
  (`NAME=oams_toolchange`). On success it returns to the starting position at
  100 mm/s and restores the positioning and extrusion modes.
- A cold-extruder guard (`can_extrude` false or below
  `minimum_extrude_temperature`) and, for `T0`-`T19`, an unknown filament group
  check fail before any motion, including homing.
- `SAFE_UNLOAD_FILAMENT` sets `M83` before the first retract and retracts at
  `unload_speed`. `CUT_FILAMENT` issues `G90` before the cutter moves.
- Four new variables in `[gcode_macro _oams_macro_variables]`, with these
  defaults when absent: `unload_speed: 1000`,
  `extrusion_unload_additional_length: 250`, `additional_unload_speed: 5000`,
  `minimum_extrude_temperature: 0`.
- The macros require `[pause_resume]` and read the `extruder` and `configfile`
  status objects.

## Configuration notes

- Filament group names must be unique. If the same `[filament_group <name>]` section appears more than once (for example two `[filament_group T1]` blocks), Klipper will now stop during startup and report the duplicate so you can fix the config before printing.

## MFRC522 RFID readers

Firmware 2.0.25 exposes Klipper standard software-SPI commands. Both readers
share SCK `PA8`, MOSI `PA9`, and MISO `PA10`; RFID A uses CS `PB3` and
reset/NPD `PD2`, while RFID B uses CS `PB2` and reset/NPD `PB0`.

One `mfrc522.py` driver owns both reader endpoints and serializes them through
one command queue. Configure the Bambu master key locally to enable per-sector
authentication and filament decoding:

```ini
[bambu_lab_tag_processor]
key: <32_HEX_CHARACTERS>

[mfrc522 openams]
oams: 1
cs_pin_a: oams_mcu1:PB3
reset_pin_a: oams_mcu1:PD2
cs_pin_b: oams_mcu1:PB2
reset_pin_b: oams_mcu1:PB0
spi_software_sclk_pin: oams_mcu1:PA8
spi_software_mosi_pin: oams_mcu1:PA9
spi_software_miso_pin: oams_mcu1:PA10
spi_speed: 100000
```

`OAMSM_RFID_READ OAMS=1 RFID_CARD=0` performs an immediate read and reports the current card data plus `LAST_READ_STATUS`. `MFRC522_QUERY READER=rfid_a` remains available as a reader-name diagnostic. Optional
`read_block`, `key`, and `key_b` settings enable an authenticated MIFARE block
read.

## Credits  

This project was made by knight.rad_iant on Discord.

---
