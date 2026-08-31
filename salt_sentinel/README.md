# Salt Sentinel — rover software

Non-contact salt-attack monitoring rover. Two processors split along a timing
boundary: the Pi owns vision and scoring, the ESP32 owns motor PWM, servos and
the safety watchdog. One USB cable carries power and serial between them.

```
esp32/salt_sentinel_drive/   Arduino firmware — motors, servos, watchdog
pi/saltsentinel/
  config.py       every tunable: I2C map, GPIO map, optics, geometry
  drive.py        serial client + keepalive thread
  sensors.py      ToF ranging (XSHUT addressing), climate, IMU, current
  thermal.py      AMG8833 flat-field + in-frame differential
  camera.py       picamera2 with exposure/AWB/focus LOCKED
  photometric.py  4-light solve -> albedo + normals, cross-visit registration
  geometry.py     thermal <-> camera mapping from measured arm range
  leds.py         LED ring via ULN2803, one light at a time
  patrol.py       wall-following, far pass + adaptive near pass
  store.py        per-station records tagged with their conditions
pi/cli.py         selftest / teleop / calibration / patrol / geometry
```

## Quick start

```bash
bash pi/setup_pi.sh && sudo reboot
source ~/ss-venv/bin/activate
cd pi
python cli.py --sim selftest     # no hardware needed
python cli.py --sim geometry     # standoff and overlay maths
python cli.py selftest           # real rover
```

## Bring-up order

1. `python cli.py --sim selftest` — software sane
2. Flash `esp32/salt_sentinel_drive` — expect `EV READY` on serial
3. `python cli.py selftest` — telemetry arriving
4. `python cli.py teleop` — manual override, rover on blocks first
5. `python cli.py calib-thermal` — two-point flat field
6. `python cli.py calib-camera` — meter once, freeze forever
7. `python cli.py station` — one capture, no driving
8. `python cli.py patrol`

## Current hardware assumptions

| | state |
|---|---|
| e-stop | removed. The 500 ms firmware watchdog is the only automatic stop |
| pack voltage divider | NOT fitted. `HAVE_VPACK_SENSE 0`, duty capped at 71% |
| ToF fitted | `wall_a`, `wall_b` on the arm. No `front` → no corner detection |
| rain sensors | not wired. `Patrol.raining` is operator-set |
| LED switching | ULN2803A, 3.3 V logic direct, no MOSFETs |

Each is a one-line change in `config.py` or the `.ino` when the hardware lands.

## Five things that will bite you

**ToF addresses are volatile.** Every VL53L0X ships at 0x29. `SensorHub` wakes
them one at a time via XSHUT and reassigns them — every boot, forever. If
`i2cdetect` shows 0x29, that sequence did not run.

**The thermal differential needs a dry reference in frame.** At 250 mm the
frame is 289 mm wide — about one brick — so a large damp patch fills it and the
differential subtracts to zero, going blind on the strongest signal. Run the
thermal pass at ~1 m. `python cli.py geometry` prints where the boundary falls.

**Efflorescence needs an absolute white reference.** `calibrate_white()` images
the calibration card before the pass. Without it `bright_fraction` returns NaN
on purpose: a percentile threshold reports the same value for a clean wall and
a crusted one, silently destroying the growth-rate channel.

**The overlay needs the range measured on the ARM.** Parallax between thermal
and camera is 322 px at 250 mm — 16% of frame width — and scales with distance.
Assume the range and the two frames measure different bricks.

**Never re-run `calib-camera` between visits.** Autofocus and auto-exposure are
useful once, to pick values; after that they make the camera measure itself.

## Pi 5 specifics

- `RPi.GPIO` and `pigpio` do not work — GPIO is behind the RP1 chip. Use
  `gpiozero` with the `lgpio` backend, as this code does.
- Check `vcgencmd get_throttled` **after a full processing load**, not at idle.
  Anything but `0x0` means the supply is inadequate; bit 16 is sticky.
- Put the `/dev/serial/by-id/...` path in `SERIAL_PORT` — it never renumbers.
- Fit the RTC coin cell. No network at the site means no NTP, and this is a
  time-series project.
