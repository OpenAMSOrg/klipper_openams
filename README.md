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
The extension deliberately accepts only tested Klipper baselines and requires
Python 3.9 or newer in Klipper's environment. Follow its README before adding
`[gco_routines]` ahead of all macro sections. Opt individual macros in with
`render_mode: ordered`; unspecified macros keep stock whole-template rendering.
The sample single-FPS macros in that repository include printer-specific cutter
geometry and must be adapted, not copied over a working printer configuration.

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
