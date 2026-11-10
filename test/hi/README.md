# Creality Hi test data

- `mcu.dict`, `nozzle_mcu.dict`, `bed_mcu.dict`: Klipper data dictionaries of the three MCUs of the
  Creality Hi with stock firmware V1.1.0.60 (main `mcu0_110_G32-mcu0_020_000`, nozzle
  `noz0_110_G30-noz0_019_000`, bed read on the printer). Copied from Creality-Hi-CFW
  `data/derived/mcu-dictionaries/hi/`.
- `phase1.gcode`: commands the phase 1 config (`config/hi/printer.cfg`) must accept.

Used by `test/test_hi_config.py`, which runs Kalico in batch mode against the real dictionaries and also
checks that homing and every X/Y move are rejected.
