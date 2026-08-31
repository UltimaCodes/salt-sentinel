"""Serial link to the ESP32 drive controller.

The firmware stops the motors if it doesn't hear from us within 500ms, so
this client runs a keepalive thread - if this process crashes or blocks,
the rover coasts to a stop on its own. That watchdog is the only automatic
stop the rover has now that the hardware e-stop is gone.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from . import config as cfg

# Mirrors SAFE_CAP in the firmware: assume a full 4S pack (16.8 V) and cap duty
# so 12 V motors never see more than 12 V. Used when no divider is fitted.
SAFE_DUTY_CAP = 12.0 / 16.8

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - allows laptop development
    serial = None


@dataclass
class Telemetry:
    pack_v: float = -1.0        # -1 = no divider fitted on the ESP32
    enabled: bool = False
    vpack_fault: bool = False
    duty_cap: float = 0.0       # what the firmware is actually limiting to
    left: int = 0
    right: int = 0
    pan_us: int = 1500
    tilt_us: int = 1500
    enc_l: int = 0
    enc_r: int = 0
    stamp: float = field(default_factory=time.time)

    @property
    def stale(self) -> bool:
        return (time.time() - self.stamp) > 1.0

    @property
    def has_vpack(self) -> bool:
        return self.pack_v >= 0.0


class Drive:
    def __init__(self, port: str = cfg.SERIAL_PORT, baud: int = cfg.SERIAL_BAUD,
                 simulate: bool = False):
        self.simulate = simulate or serial is None
        self.telemetry = Telemetry()
        self._lock = threading.Lock()
        self._left = 0
        self._right = 0
        self._run = False
        self._events: list[str] = []
        self._ser = None
        if not self.simulate:
            self._ser = serial.Serial(port, baud, timeout=0.2)
            time.sleep(2.0)          # ESP32 resets when the port opens
            self._ser.reset_input_buffer()

    # ------------------------------------------------------------- lifecycle
    def start(self):
        self._run = True
        self._rx = threading.Thread(target=self._reader, daemon=True)
        self._tx = threading.Thread(target=self._keepalive, daemon=True)
        self._rx.start()
        self._tx.start()
        return self

    def close(self):
        self._run = False
        try:
            self.stop()
            self.enable(False)
        except Exception:
            pass
        time.sleep(0.1)
        if self._ser:
            self._ser.close()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()

    # --------------------------------------------------------------- command
    def _send(self, line: str):
        if self.simulate:
            return
        with self._lock:
            self._ser.write((line + "\n").encode())

    def enable(self, on: bool = True):
        self._send(f"E {1 if on else 0}")

    def drive(self, left: int, right: int):
        """Set both channels. Both left motors share pins, as do both right."""
        self._left = max(-1000, min(1000, int(left)))
        self._right = max(-1000, min(1000, int(right)))
        self._send(f"D {self._left} {self._right}")

    def stop(self):
        self.drive(0, 0)

    def tank(self, forward: int, differential: int):
        """forward = base speed, differential = + steers right."""
        d = max(-cfg.MAX_DIFF, min(cfg.MAX_DIFF, int(differential)))
        self.drive(forward + d, forward - d)

    def servos(self, pan_us: int, tilt_us: int):
        self._send(f"S {int(pan_us)} {int(tilt_us)}")

    def ping(self):
        self._send("P")

    def pop_events(self) -> list[str]:
        ev, self._events = self._events, []
        return ev

    # --------------------------------------------------------------- threads
    def _keepalive(self):
        """Re-send the current command so the firmware watchdog stays fed."""
        while self._run:
            self._send(f"D {self._left} {self._right}")
            time.sleep(cfg.DRIVE_KEEPALIVE_S)

    def _reader(self):
        while self._run:
            if self.simulate:
                time.sleep(0.1)
                # mirror what the firmware actually reports so the simulation
                # is representative: no divider fitted -> fixed safe duty cap
                self.telemetry.stamp = time.time()
                self.telemetry.duty_cap = SAFE_DUTY_CAP
                self.telemetry.left = self._left
                self.telemetry.right = self._right
                continue
            try:
                raw = self._ser.readline().decode(errors="ignore").strip()
            except Exception:
                time.sleep(0.1)
                continue
            if not raw:
                continue
            if raw.startswith("ST "):
                self._parse_telemetry(raw[3:])
            elif raw.startswith("EV "):
                self._events.append(raw[3:])

    def _parse_telemetry(self, body: str):
        t = Telemetry(stamp=time.time())
        for tok in body.split():
            if "=" not in tok:
                continue
            k, v = tok.split("=", 1)
            try:
                if k == "v":
                    t.pack_v = float(v)
                elif k == "en":
                    t.enabled = v == "1"
                elif k == "vf":
                    t.vpack_fault = v == "1"
                elif k == "cap":
                    t.duty_cap = float(v)
                elif k == "L":
                    t.left = int(v)
                elif k == "R":
                    t.right = int(v)
                elif k == "pan":
                    t.pan_us = int(v)
                elif k == "tilt":
                    t.tilt_us = int(v)
                elif k == "encL":
                    t.enc_l = int(v)
                elif k == "encR":
                    t.enc_r = int(v)
            except ValueError:
                continue
        self.telemetry = t
