"""Mapping thermal pixels onto camera pixels.

The AMG8833 and camera view the wall from a small baseline apart, and the
offset between them in PIXELS scales with distance:

    parallax_px = baseline_mm * (camera_px_width / camera_footprint_mm)

At 250mm standoff / 40mm baseline that's 322px (~16% of frame) - guess the
distance wrong by 50mm and the overlay is off by 80px, 25% scale error.

The ToF pair that measures this distance is chassis-mounted, not on the
static arm where the thermal array and camera actually sit (see sensors.py /
config.ARM_TO_CHASSIS_OFFSET_MM). Every distance this module receives is
expected to already be arm-to-wall range, offset already applied by the
caller - not raw chassis range.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, asdict

import numpy as np

from . import config as cfg


# --------------------------------------------------------------- footprints
def footprint_mm(distance_mm: float, fov_deg: float) -> float:
    """Width of the area a sensor sees at a given distance."""
    return 2.0 * distance_mm * math.tan(math.radians(fov_deg / 2.0))


def slant_range_mm(perpendicular_mm: float, tilt_deg: float) -> float:
    """Distance along the optical axis when tilted off normal - ignoring
    this puts a 6% scale error into every reading at 20deg of tilt."""
    return perpendicular_mm / max(1e-6, math.cos(math.radians(tilt_deg)))


def parallax_px(distance_mm: float, baseline_mm: float,
                cam_px_width: int, cam_fov_deg: float) -> float:
    """Pixel offset between the thermal and camera views of the same point."""
    return baseline_mm * (cam_px_width / footprint_mm(distance_mm, cam_fov_deg))


@dataclass
class OverlayGeometry:
    distance_mm: float
    thermal_footprint_mm: float
    camera_footprint_mm: float
    px_per_mm: float
    parallax_px: float
    usable_thermal_cols: float      # thermal columns that fall inside the image
    fully_covered: bool

    def as_dict(self):
        return asdict(self)


def overlay_geometry(distance_mm: float,
                     baseline_mm: float = cfg.THERMAL_CAM_BASELINE_MM,
                     cam_px_width: int = cfg.CAM_PX_WIDTH,
                     cam_fov_deg: float = cfg.CAM_FOV_DEG) -> OverlayGeometry:
    wt = footprint_mm(distance_mm, cfg.AMG_FOV_DEG)
    wc = footprint_mm(distance_mm, cam_fov_deg)
    ppm = cam_px_width / wc
    # the thermal sensor is the wider of the two, so its outer columns see wall
    # the camera never sees - a FOV mismatch, independent of distance
    usable = cfg.AMG_COLS * min(1.0, wc / wt)
    return OverlayGeometry(
        distance_mm=distance_mm,
        thermal_footprint_mm=wt,
        camera_footprint_mm=wc,
        px_per_mm=ppm,
        parallax_px=baseline_mm * ppm,
        usable_thermal_cols=usable,
        fully_covered=wc >= wt,
    )


# ------------------------------------------------------------------ mapping
def thermal_pixel_to_camera(row: int, col: int, distance_mm: float,
                            cam_px_size: tuple = (cfg.CAM_PX_WIDTH, cfg.CAM_PX_HEIGHT),
                            baseline_mm: float = cfg.THERMAL_CAM_BASELINE_MM,
                            cam_fov_deg: float = cfg.CAM_FOV_DEG):
    """Where thermal pixel (row, col) lands in the camera image, as
    (x0, y0, x1, y1) - a rectangle, since one thermal pixel covers many
    camera pixels."""
    w_px, h_px = cam_px_size
    g = overlay_geometry(distance_mm, baseline_mm, w_px, cam_fov_deg)

    mm_per_thermal_px = g.thermal_footprint_mm / cfg.AMG_COLS
    x_mm = (col - (cfg.AMG_COLS - 1) / 2.0) * mm_per_thermal_px
    y_mm = (row - (cfg.AMG_ROWS - 1) / 2.0) * mm_per_thermal_px

    cx = w_px / 2.0 + (x_mm + baseline_mm) * g.px_per_mm
    cy = h_px / 2.0 + y_mm * g.px_per_mm
    half = (mm_per_thermal_px * g.px_per_mm) / 2.0
    return (cx - half, cy - half, cx + half, cy + half)


def thermal_to_camera_map(distance_mm: float,
                          cam_px_size: tuple = (cfg.CAM_PX_WIDTH, cfg.CAM_PX_HEIGHT),
                          baseline_mm: float = cfg.THERMAL_CAM_BASELINE_MM,
                          cam_fov_deg: float = cfg.CAM_FOV_DEG) -> np.ndarray:
    """Index map (h, w, 2) of the (row, col) thermal pixel under every
    camera pixel; -1 outside the thermal footprint. For an MSX-style
    overlay registered by measured distance, not an assumed one."""
    w_px, h_px = cam_px_size
    g = overlay_geometry(distance_mm, baseline_mm, w_px, cam_fov_deg)
    mm_per_thermal_px = g.thermal_footprint_mm / cfg.AMG_COLS

    xs = (np.arange(w_px) - w_px / 2.0) / g.px_per_mm - baseline_mm
    ys = (np.arange(h_px) - h_px / 2.0) / g.px_per_mm
    cols = np.round(xs / mm_per_thermal_px + (cfg.AMG_COLS - 1) / 2.0).astype(int)
    rows = np.round(ys / mm_per_thermal_px + (cfg.AMG_ROWS - 1) / 2.0).astype(int)
    cols[(cols < 0) | (cols >= cfg.AMG_COLS)] = -1
    rows[(rows < 0) | (rows >= cfg.AMG_ROWS)] = -1

    out = np.empty((h_px, w_px, 2), dtype=np.int16)
    out[..., 0] = rows[:, None]
    out[..., 1] = cols[None, :]
    out[(out[..., 0] < 0) | (out[..., 1] < 0)] = -1
    return out


def overlay(thermal: np.ndarray, distance_mm: float,
            cam_px_size: tuple = (cfg.CAM_PX_WIDTH, cfg.CAM_PX_HEIGHT),
            **kw) -> np.ndarray:
    """Expand an 8x8 thermal frame to camera resolution, NaN where uncovered."""
    idx = thermal_to_camera_map(distance_mm, cam_px_size, **kw)
    out = np.full(idx.shape[:2], np.nan)
    ok = (idx[..., 0] >= 0) & (idx[..., 1] >= 0)
    out[ok] = thermal[idx[..., 0][ok], idx[..., 1][ok]]
    return out


def check(distance_mm: float, tilt_deg: float = 0.0) -> list[str]:
    """Warnings a station capture should record alongside its data."""
    msgs = []
    d = slant_range_mm(distance_mm, tilt_deg)
    g = overlay_geometry(d)
    if not g.fully_covered:
        msgs.append(f"thermal FOV ({cfg.AMG_FOV_DEG:.0f} deg) exceeds camera FOV "
                    f"({cfg.CAM_FOV_DEG:.0f} deg): only {g.usable_thermal_cols:.1f} of "
                    f"{cfg.AMG_COLS} thermal columns have camera data")
    if g.parallax_px > 0.10 * cfg.CAM_PX_WIDTH:
        msgs.append(f"parallax {g.parallax_px:.0f} px is "
                    f"{g.parallax_px/cfg.CAM_PX_WIDTH*100:.0f}% of frame width - "
                    f"overlay depends entirely on the measured arm range")
    if abs(tilt_deg) > 5.0:
        msgs.append(f"tilt {tilt_deg:.0f} deg stretches slant range to {d:.0f} mm "
                    f"from {distance_mm:.0f} mm nominal ({(d/distance_mm-1)*100:.1f}% scale)")
    return msgs
