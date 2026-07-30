# OpenAMS for Klipper  
OpenAMS Klipper Plugin

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
./install-openams.sh [-k <klipper path>] [-s <klipper service name>] [-c <configuration path>]  
```

## Configuration notes

- Filament group names must be unique. If the same `[filament_group <name>]` section appears more than once (for example two `[filament_group T1]` blocks), Klipper will now stop during startup and report the duplicate so you can fix the config before printing.

## MFRC522 RFID readers

Firmware 2.0.25 exposes Klipper standard software-SPI commands. Both readers
share SCK `PA8`, MOSI `PA9`, and MISO `PA10`; RFID A uses CS `PB3` and
reset/NPD `PD2`, while RFID B uses CS `PB2` and reset/NPD `PB0`.

Standalone reader sections use the `mfrc522.py` driver. Configure the Bambu
master key locally to enable per-sector authentication and filament decoding:

```ini
[bambu_lab_tag_processor]
key: <32_HEX_CHARACTERS>

[mfrc522 rfid_a]
oams: 1
rfid_card: 0
cs_pin: oams_mcu1:PB3
reset_pin: oams_mcu1:PD2
spi_software_sclk_pin: oams_mcu1:PA8
spi_software_mosi_pin: oams_mcu1:PA9
spi_software_miso_pin: oams_mcu1:PA10
spi_speed: 750000

[mfrc522 rfid_b]
oams: 1
rfid_card: 1
cs_pin: oams_mcu1:PB2
reset_pin: oams_mcu1:PB0
spi_software_sclk_pin: oams_mcu1:PA8
spi_software_mosi_pin: oams_mcu1:PA9
spi_software_miso_pin: oams_mcu1:PA10
spi_speed: 750000
```

`OAMSM_RFID_READ OAMS=1 RFID_CARD=0` performs an immediate read and reports the current card data plus `LAST_READ_STATUS`. `MFRC522_QUERY READER=rfid_a` remains available as a reader-name diagnostic. Optional
`read_block`, `key`, and `key_b` settings enable an authenticated MIFARE block
read. If AFC owns the readers through `[AFC_OpenAMS_rfid ...]`, do not also
configure these standalone sections; AFC uses the same Klipper SPI transport.

## Credits  

This project was made by knight.rad_iant on Discord.

---