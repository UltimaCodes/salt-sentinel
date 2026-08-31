#!/usr/bin/env bash
# Salt Sentinel - Raspberry Pi 5 setup.  Run:  bash setup_pi.sh
set -euo pipefail

echo "== enabling interfaces =="
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_camera 0 2>/dev/null || true

CONFIG=/boot/firmware/config.txt
[ -f "$CONFIG" ] || CONFIG=/boot/config.txt
add_line() { grep -qxF "$1" "$CONFIG" || echo "$1" | sudo tee -a "$CONFIG" >/dev/null; }

# 100 kHz: 400 kHz is unreliable over a metre of cable up a sensor arm.
add_line "dtparam=i2c_arm=on,i2c_arm_baudrate=100000"
# Clean-shutdown button. Hard power cuts corrupt the SD card, and that is one
# of the most common ways a working robot dies the night before a competition.
add_line "dtoverlay=gpio-shutdown,gpio_pin=26,active_low=1,gpio_pull=up"
# Allow full USB current when the supply is not a detected 5 A PSU.
add_line "usb_max_current_enable=1"

echo "== updating package index =="
# apt-get update MUST run before install, or every package name below can
# come back "Unable to locate package" even when the name is correct - the
# error looks like a typo but is really a stale/empty index.
sudo apt-get update
echo "-- confirm the index actually populated --"
apt-cache search picamera2 | grep -q picamera2 || {
  echo "!! apt-cache still finds nothing after update."
  echo "!! Check /etc/apt/sources.list and internet access before continuing."
  exit 1
}

echo "== packages =="
# Keep this list minimal and apt-only for things that MUST come from apt
# (picamera2 - do not pip install it, it will not work). Everything else
# (gpiozero, lgpio, opencv, pyserial, numpy) comes from requirements.txt via
# pip instead, because apt package names for those vary by Pi OS release and
# are a common source of "Unable to locate package" on a machine that isn't
# running the exact OS version a guide was written against.
sudo apt-get install -y python3-pip python3-venv python3-picamera2 i2c-tools

echo "== python environment =="
python3 -m venv --system-site-packages ~/ss-venv
source ~/ss-venv/bin/activate
pip install --upgrade pip
pip install -r "$(dirname "$0")/requirements.txt"

echo "== checks =="
echo "-- i2c bus (expect 0x30-0x32 AFTER ToF init, 0x29 before) --"
i2cdetect -y 1 || true
echo "-- power supply (must be 0x0 after a full load, not at idle) --"
vcgencmd get_throttled
echo "-- serial ports --"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "  none - is the ESP32 plugged in?"
echo "-- camera --"
rpicam-hello --list-cameras 2>&1 || libcamera-hello --list-cameras 2>&1 || true

cat <<'NOTE'

Done. Next:
  source ~/ss-venv/bin/activate
  cd salt_sentinel/pi
  python cli.py --sim selftest
  python cli.py selftest

If "python cli.py selftest" (no --sim) ever prints sensor numbers instead of
a clear RuntimeError when a library is missing, that is a bug - it should
fail loudly, never silently substitute simulated data. It was fixed to do
this; if you see it happen again, something regressed.

Reboot once so config.txt takes effect.

Put the /dev/serial/by-id/... path into SERIAL_PORT in config.py - it never
renumbers, unlike /dev/ttyUSB0.

Camera: this rig uses the OV5647 (Camera Module 1, "Rev 1.3") via an
adapter cable, which is FIXED FOCUS. There is no software autofocus for it.
Focus it by hand - see the docstring in saltsentinel/camera.py - and never
touch the lens ring again once it's set.
NOTE
