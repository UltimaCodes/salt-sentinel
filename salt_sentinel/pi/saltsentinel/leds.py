"""Wall fill light - a single LED ring, switched through one transistor
off the Pi's GPIO. Just gives the camera consistent lighting; not a
photometric rig, so it only supports on/off, not directional sequencing.

Pi 5 note: RPi.GPIO/pigpio don't work here (GPIO sits behind the RP1 chip) -
gpiozero with the lgpio backend does.
"""

from __future__ import annotations

import time
from contextlib import contextmanager

from . import config as cfg

try:
    from gpiozero import DigitalOutputDevice
    HW = True
    HW_ERR = None
except Exception as e:  # pragma: no cover
    HW = False
    HW_ERR = e


class LedRing:
    def __init__(self, pin: int = cfg.LED_RING_PIN, simulate: bool = False,
                 settle_s: float = 0.06):
        if not simulate and not HW:
            raise RuntimeError(
                "real hardware requested (no --sim) but gpiozero is not "
                f"importable. Import error: {HW_ERR!r}. pip install gpiozero "
                "lgpio, or pass --sim.")
        self.simulate = simulate
        self.settle_s = settle_s
        self.pin = pin
        self._dev = None if simulate else DigitalOutputDevice(pin, initial_value=False)

    def on(self):
        if not self.simulate:
            self._dev.on()

    def off(self):
        if not self.simulate:
            self._dev.off()

    @contextmanager
    def lit(self):
        """Ring on for one capture, off again afterwards."""
        self.on()
        time.sleep(self.settle_s)
        try:
            yield
        finally:
            self.off()

    def close(self):
        self.off()
        if self._dev:
            try:
                self._dev.close()
            except Exception:
                pass
