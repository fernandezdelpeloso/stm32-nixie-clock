# STM32 Nixie Clock (R|Z568M)

**TL;DR:** One-tube Nixie clock (Dalibor Farny R|Z568M) on an STM32 Cortex-M4F with GPS time sync, RTC backup, and anti–cathode-poisoning routine.

[▶ Short demo (MP4)](media/videos/bright.mp4)
![Thumbnail](media/thumbs/bright.gif)

## Hardware
- MCU: STM32L4A6RGT7
- Level shifting: PCA9306 (custom module)
- HV driver: K155ID1 (5 V logic)
- Time: NEO-M8N (UTC) + DS3231 (backup)

### Files in this repo
- Schematics (PDF): `hardware/exports/schematic/`
- PCB (design): `hardware/exporta/pcb`
- 3D (PCB model): `hardware/exports/3d/`
- BOM (CSV + iBOM): `hardware/exports/bom/`
- Native KiCad project: `hardware/kicad/`

## Firmware
`firmware/` (STM32CubeIDE project). Roadmap: GPS/NMEA parse, RTC sync, presence sensor → anti-poisoning routine, optional FreeRTOS + SystemView.

## Safety
**High voltage present.** Observe clearance/creepage and safe handling.

## License
MIT
				
