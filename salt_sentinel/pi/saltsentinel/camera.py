"""Camera capture with everything locked.

Auto-exposure/white-balance are harmful here since every output is a
cross-visit comparison, not a single shot. This rig's OV5647 is fixed-focus
(no AfMode) - focus is set once by physically turning the lens ring and
never touched again. calib-camera only locks exposure/WB/gain, the things
this sensor can actually control in software. A future AF-capable camera
module works automatically: apply_lock() and autotune_once() detect AF
support at runtime.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass

import numpy as np

from . import config as cfg

try:
    from picamera2 import Picamera2
    HW = True
    HW_ERR = None
except Exception as e:  # pragma: no cover
    HW = False
    HW_ERR = e

try:
    from libcamera import controls as _lc_controls
except Exception:
    _lc_controls = None

LOCK_FILE = cfg.CALIB / "camera_lock.json"


@dataclass
class CameraLock:
    exposure_us: int = 8000
    analogue_gain: float = 1.5
    colour_gains: tuple = (1.8, 1.6)
    lens_position: float | None = None    # None on a fixed-focus sensor (OV5647)
    resolution: tuple = (cfg.CAM_PX_WIDTH, cfg.CAM_PX_HEIGHT)

    def save(self):
        cfg.CALIB.mkdir(parents=True, exist_ok=True)
        LOCK_FILE.write_text(json.dumps(self.__dict__, indent=2, default=list))

    @classmethod
    def load(cls) -> "CameraLock":
        if LOCK_FILE.exists():
            d = json.loads(LOCK_FILE.read_text())
            d["colour_gains"] = tuple(d["colour_gains"])
            d["resolution"] = tuple(d["resolution"])
            return cls(**d)
        return cls()


def lens_position_for(standoff_mm: float) -> float:
    """libcamera LensPosition is in dioptres = 1 / distance_in_metres.
    Only meaningful on a sensor with software AF (e.g. Camera Module 3)."""
    return 1000.0 / max(60.0, standoff_mm)


class Camera:
    def __init__(self, simulate: bool = False, lock: CameraLock | None = None):
        if not simulate and not HW:
            raise RuntimeError(
                "real hardware requested (no --sim) but picamera2 is not "
                "importable, so this would otherwise have SILENTLY returned "
                f"simulated frames instead of failing. Import error: {HW_ERR!r}. "
                "picamera2 must come from apt (python3-picamera2), not pip - "
                "see PI_SETUP.md.")
        self.simulate = simulate
        self.lock = lock or CameraLock.load()
        self._cam = None
        self._has_af = False
        if self.simulate:
            return
        self._cam = Picamera2()
        conf = self._cam.create_still_configuration(
            main={"size": self.lock.resolution, "format": "RGB888"})
        self._cam.configure(conf)
        self._cam.start()
        time.sleep(1.0)
        # OV5647 has no "AfMode" key here at all - this is how we tell a
        # fixed-focus sensor from one with real autofocus, instead of
        # guessing from the module name.
        self._has_af = "AfMode" in self._cam.camera_controls
        if not self._has_af:
            print("Camera: no AfMode control - fixed-focus sensor (OV5647). "
                 "Focus is set by physically turning the lens ring and must "
                 "never be touched again.")
        self.apply_lock()

    def apply_lock(self):
        if self.simulate:
            return
        controls = {
            "AeEnable": False,
            "AwbEnable": False,
            "ExposureTime": int(self.lock.exposure_us),
            "AnalogueGain": float(self.lock.analogue_gain),
            "ColourGains": tuple(self.lock.colour_gains),
        }
        if self._has_af and self.lock.lens_position is not None and _lc_controls:
            controls["AfMode"] = _lc_controls.AfModeEnum.Manual
            controls["LensPosition"] = float(self.lock.lens_position)
        self._cam.set_controls(controls)
        time.sleep(0.4)

    def autotune_once(self, standoff_mm: float = cfg.STANDOFF_NEAR_MM) -> CameraLock:
        """Meter the scene once, freeze the result, store it. Run once at
        the validation wall; re-running between visits breaks comparability."""
        if self.simulate:
            return self.lock
        self._cam.set_controls({"AeEnable": True, "AwbEnable": True})
        time.sleep(2.5)
        md = self._cam.capture_metadata()
        self.lock.exposure_us = int(md.get("ExposureTime", 8000))
        self.lock.analogue_gain = float(md.get("AnalogueGain", 1.5))
        cg = md.get("ColourGains", (1.8, 1.6))
        self.lock.colour_gains = (float(cg[0]), float(cg[1]))
        if self._has_af:
            self.lock.lens_position = lens_position_for(standoff_mm)
        self.lock.save()
        self.apply_lock()
        return self.lock

    def grab_gray(self) -> np.ndarray:
        """One frame as float luminance."""
        if self.simulate:
            rng = np.random.default_rng(0)
            base = rng.normal(120, 8, (240, 320))
            base[90:150, 130:210] += 55        # a bright patch standing in for crust
            return base
        rgb = self._cam.capture_array("main").astype(np.float64)
        return 0.2126 * rgb[..., 2] + 0.7152 * rgb[..., 1] + 0.0722 * rgb[..., 0]

    def grab_rgb(self) -> np.ndarray:
        if self.simulate:
            g = self.grab_gray()
            return np.dstack([g, g, g]).astype(np.uint8)
        return self._cam.capture_array("main")

    def close(self):
        if self._cam:
            try:
                self._cam.stop()
            except Exception:
                pass
