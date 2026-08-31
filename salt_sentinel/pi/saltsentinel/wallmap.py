"""Wall panorama + thermal heatmap overlay.

Stitching and colorizing are separate functions so real sensor data can
swap in without touching either. build_heat_strip() doesn't care whether
the moisture numbers came from a real ThermalCamera or demo_scenario.py.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class WallmapResult:
    image_bgr: np.ndarray
    stitched_from: int          # how many source photos went in
    out_path: str = ""


# ------------------------------------------------------------------ stitch
def stitch_or_single(photo_paths: list[str]) -> tuple[np.ndarray, int]:
    """cv2.Stitcher panorama for 2+ overlapping photos, otherwise the one
    photo as-is."""
    imgs = [cv2.imread(p) for p in photo_paths]
    imgs = [i for i in imgs if i is not None]
    if not imgs:
        raise FileNotFoundError(f"none of {photo_paths} could be read")
    if len(imgs) == 1:
        return imgs[0], 1

    stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    status, pano = stitcher.stitch(imgs)
    if status != cv2.Stitcher_OK:
        return imgs[0], 1   # fall back to the first photo rather than fail outright
    return pano, len(imgs)


def normalize_like_capture(img: np.ndarray, target_width: int = 2028) -> np.ndarray:
    """Crop to a wide letterboxed frame and flatten exposure/contrast a bit,
    so a handheld photo reads closer to a fixed-standoff capture."""
    h, w = img.shape[:2]
    target_ratio = 16 / 7.0
    cur_ratio = w / h
    if cur_ratio > target_ratio:
        new_w = int(h * target_ratio)
        x0 = (w - new_w) // 2
        img = img[:, x0:x0 + new_w]
    else:
        new_h = int(w / target_ratio)
        y0 = max(0, (h - new_h) // 3)   # bias up: more sky/ground than side margin
        img = img[y0:y0 + new_h, :]

    scale = target_width / img.shape[1]
    img = cv2.resize(img, (target_width, int(img.shape[0] * scale)), interpolation=cv2.INTER_CUBIC)

    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    img = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    return img


# ------------------------------------------------------------------ heatmap
def build_heat_strip(station_odo_mm: list[float], station_cooling_c: list[float],
                     station_damp_height_mm: list[float],
                     width_px: int, height_px: int, wall_length_mm: float | None = None,
                     spot_threshold: float = 0.15, seed: int = 7) -> np.ndarray:
    """Per-station cooling values -> discrete soft-edged spots, 0..1, shape
    (height_px, width_px) - real damp patches on a wall are localised, not
    a uniform band, so this places one blob per station that actually has a
    signal (skips near-zero/dry stations, so it stays sparse) rather than
    smearing every reading across a continuous strip. Position within the
    frame gets a little jitter so spots don't look mechanically aligned;
    size scales with that station's damp_height_mm."""
    n = len(station_odo_mm)
    heat = np.zeros((height_px, width_px), dtype=np.float32)
    if n == 0:
        return heat
    wall_length_mm = wall_length_mm or (max(station_odo_mm) + 400.0)

    cooling = np.clip(np.array(station_cooling_c, dtype=np.float32), 0.0, None)
    peak = float(cooling.max()) or 1.0
    cooling_n = cooling / peak
    damp_h = np.array(station_damp_height_mm, dtype=np.float32)

    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:height_px, 0:width_px].astype(np.float32)

    for odo, w, dh in zip(station_odo_mm, cooling_n, damp_h):
        if w < spot_threshold:
            continue   # dry/control station - no visible spot, keeps it sparse
        x_center = (odo / wall_length_mm) * width_px
        y_center = height_px * rng.uniform(0.4, 0.62)
        size = 0.6 + 0.4 * np.clip(dh / 300.0, 0.0, 1.0)
        radius_x = width_px * rng.uniform(0.045, 0.075) * size
        radius_y = height_px * rng.uniform(0.12, 0.19) * size
        d2 = ((xx - x_center) / radius_x) ** 2 + ((yy - y_center) / radius_y) ** 2
        blob = w * np.exp(-1.6 * d2)
        heat = np.maximum(heat, blob)   # max-combine: overlapping spots don't wash out
    return heat


def render_thermal_style(wall_bgr: np.ndarray, heat_cooling: np.ndarray,
                         texture_weight: float = 0.16) -> np.ndarray:
    """Full-frame false-colour render. heat_cooling (0..1, already resized
    to wall_bgr's shape) maps straight to palette value - cold/damp stays
    the dark end, warm/dry the bright end, matching how a real thermal
    camera reads this scene."""
    h, w = wall_bgr.shape[:2]
    gray = cv2.cvtColor(wall_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0
    texture = gray - cv2.GaussianBlur(gray, (0, 0), sigmaX=12)

    # gentle large-scale drift so the background isn't perfectly flat,
    # without inventing spots that don't match any station reading
    rng = np.random.default_rng(2)
    drift = cv2.resize(rng.normal(0.5, 0.018, (3, 6)).astype(np.float32), (w, h),
                       interpolation=cv2.INTER_CUBIC)

    field = drift - heat_cooling * 0.62 + texture * texture_weight
    field = np.clip(field, 0.0, 1.0)

    field_u8 = (field * 255).astype(np.uint8)
    colored = cv2.applyColorMap(field_u8, cv2.COLORMAP_INFERNO)

    grain = rng.normal(0, 3, colored.shape).astype(np.int16)
    out = np.clip(colored.astype(np.int16) + grain, 0, 255).astype(np.uint8)
    return out


def render(photo_paths: list[str], station_odo_mm: list[float],
          station_cooling_c: list[float], station_damp_height_mm: list[float],
          out_path: str) -> WallmapResult:
    wall, n_src = stitch_or_single(photo_paths)
    wall = normalize_like_capture(wall)
    h, w = wall.shape[:2]
    heat = build_heat_strip(station_odo_mm, station_cooling_c, station_damp_height_mm, w, h)
    heat_r = cv2.resize(heat, (w, h), interpolation=cv2.INTER_CUBIC)
    combined = render_thermal_style(wall, heat_r)
    cv2.imwrite(out_path, combined)
    return WallmapResult(image_bgr=combined, stitched_from=n_src, out_path=out_path)
