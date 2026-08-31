"""AMG8833 thermal channel.

Flat-field correction subtracts each pixel's fixed-pattern offset (measured
once against a uniform surface) since that's what doesn't cancel in a
same-frame comparison. Everything else is an in-frame differential: cooler
than the dry brick in the SAME frame, not an absolute temperature, so
drift/ambient/sky radiance cancel out.

Needs the frame to contain both damp and dry brick, or the median becomes
the damp temperature and the differential reads zero - hence the ~1m far
pass rather than getting in close.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, asdict

import numpy as np

from . import config as cfg

try:
    import board
    import busio
    import adafruit_amg88xx
    HW = True
    HW_ERR = None
except Exception as e:  # pragma: no cover
    HW = False
    HW_ERR = e

FLAT_FIELD = cfg.CALIB / "amg8833_flatfield.json"


def frame_width_mm(standoff_mm: float) -> float:
    """Field of view at a given standoff. 60 deg total."""
    return 2.0 * standoff_mm * math.tan(math.radians(cfg.AMG_FOV_DEG / 2.0))


def mm_per_pixel(standoff_mm: float) -> float:
    return frame_width_mm(standoff_mm) / cfg.AMG_COLS


@dataclass
class ThermalResult:
    dry_reference_c: float          # median of the dry rows
    moisture_index: float           # mean cooling below dry reference, deg C
    peak_cooling_c: float           # strongest single-pixel cooling
    damp_row: int                   # row where the biggest step occurs, -1 = none
    damp_height_mm: float           # that row's height in the frame
    below_ambient: bool             # strict wet test: surface cooler than air
    noise_floor_c: float            # from the frame stack, defines what is claimable
    mm_per_px: float
    stamp: float

    def as_dict(self):
        return asdict(self)


class ThermalCamera:
    def __init__(self, simulate: bool = False):
        if not simulate and not HW:
            raise RuntimeError(
                "real hardware requested (no --sim) but adafruit_amg88xx is "
                "not importable, so this would otherwise have SILENTLY "
                f"returned simulated frames instead of failing. Import error: "
                f"{HW_ERR!r}. pip install -r requirements.txt, or pass --sim.")
        self.simulate = simulate
        self.gain = np.ones((cfg.AMG_ROWS, cfg.AMG_COLS))
        self.offset = np.zeros((cfg.AMG_ROWS, cfg.AMG_COLS))
        self._load_flat_field()
        if self.simulate:
            self._amg = None
        else:
            i2c = busio.I2C(board.SCL, board.SDA, frequency=100_000)
            self._amg = adafruit_amg88xx.AMG88XX(i2c, addr=cfg.I2C_AMG8833)
            time.sleep(0.1)

    # ------------------------------------------------------------- raw reads
    def _raw(self) -> np.ndarray:
        if self.simulate:
            # synthetic wall: dry above, ~1.8 C evaporative cooling below
            f = np.full((cfg.AMG_ROWS, cfg.AMG_COLS), 27.0)
            f[4:, :] -= 1.8
            return f + np.random.normal(0, 0.05, f.shape)
        return np.array(self._amg.pixels, dtype=float)

    def frame(self) -> np.ndarray:
        """One flat-field corrected frame."""
        return (self._raw() - self.offset) * self.gain

    def stack(self, n: int = cfg.THERMAL_FRAMES):
        """Average n frames. The rover is stationary at a station, so time is
        free and noise falls as sqrt(n): 0.05 C over 30 frames -> ~0.01 C."""
        buf = []
        for _ in range(n):
            buf.append(self.frame())
            time.sleep(cfg.THERMAL_FRAME_DELAY_S)
        arr = np.stack(buf)
        return arr.mean(axis=0), arr.std(axis=0).mean() / math.sqrt(n)

    # -------------------------------------------------------- flat fielding
    def _load_flat_field(self):
        if FLAT_FIELD.exists():
            d = json.loads(FLAT_FIELD.read_text())
            self.gain = np.array(d["gain"])
            self.offset = np.array(d["offset"])

    def calibrate_flat_field(self, cool_frames: np.ndarray, warm_frames: np.ndarray,
                             cool_ref_c: float, warm_ref_c: float):
        """Two-point per-pixel gain/offset from a flat plate imaged at two
        known temperatures. Pixel-to-pixel agreement matters more than
        absolute accuracy, since everything downstream is a differential."""
        cool = np.mean(cool_frames, axis=0)
        warm = np.mean(warm_frames, axis=0)
        span = warm - cool
        span[np.abs(span) < 1e-6] = 1e-6
        self.gain = (warm_ref_c - cool_ref_c) / span
        self.offset = cool - (cool_ref_c / self.gain)
        cfg.CALIB.mkdir(parents=True, exist_ok=True)
        FLAT_FIELD.write_text(json.dumps(
            {"gain": self.gain.tolist(), "offset": self.offset.tolist(),
             "cool_ref_c": cool_ref_c, "warm_ref_c": warm_ref_c,
             "stamp": time.time()}, indent=2))
        return self.gain, self.offset

    # ------------------------------------------------------------- analysis
    def analyse(self, standoff_mm: float, ambient_c: float,
                n: int = cfg.THERMAL_FRAMES) -> ThermalResult:
        frame, noise = self.stack(n)
        rows = frame.mean(axis=1)

        # Dry reference: top rows, which the head is aimed to keep above the
        # rising-damp line. Median, not mean - one hot pixel should not move it.
        dry = float(np.median(frame[:cfg.DRY_REFERENCE_ROWS, :]))

        cooling = dry - frame            # positive = cooler than dry = wetter
        damp = cooling[cfg.DRY_REFERENCE_ROWS:, :]

        # The rising-damp line is the row with the largest downward step. That
        # height is the indicator conservators already read by eye.
        steps = np.diff(rows)
        damp_row = -1
        if len(steps) and float(np.min(steps)) < -0.4:
            damp_row = int(np.argmin(steps)) + 1
        fh = frame_width_mm(standoff_mm)
        damp_height = (cfg.AMG_ROWS - damp_row) / cfg.AMG_ROWS * fh if damp_row > 0 else 0.0

        return ThermalResult(
            dry_reference_c=dry,
            moisture_index=float(np.mean(damp)) if damp.size else 0.0,
            peak_cooling_c=float(np.max(cooling)),
            damp_row=damp_row,
            damp_height_mm=float(damp_height),
            below_ambient=bool(float(np.min(frame)) < ambient_c),
            noise_floor_c=float(noise),
            mm_per_px=mm_per_pixel(standoff_mm),
            stamp=time.time(),
        )

    @staticmethod
    def frame_spans_reference(standoff_mm: float, min_brick_mm: float = 280.0) -> bool:
        """Guard against the silent failure mode: if the frame is not wider than
        a brick there may be no dry reference in it, and the differential will
        quietly return zero instead of reporting a large damp patch."""
        return frame_width_mm(standoff_mm) >= 1.5 * min_brick_mm
