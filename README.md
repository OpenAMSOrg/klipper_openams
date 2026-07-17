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
- On Helix firmware advertising `ADC_STREAM_V1`, FPS uses the merged DMA ADC
  engine by default (`use_adc_stream: true`). Its default schedule is five
  samples at 5 ms spacing and one prompt filtered report every 100 ms. The
  plugin automatically uses legacy `analog_in` on unsupported firmware, and
  `use_adc_stream: false` is an explicit diagnostic override. If another ADC
  consumer on that MCU has not been migrated, Helix keeps all consumers on the
  legacy engine so two owners never program one physical ADC.

## Credits  

This project was made by knight.rad_iant on Discord.

---
