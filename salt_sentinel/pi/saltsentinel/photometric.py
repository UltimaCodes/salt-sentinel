"""Photometric stereo: four LED-lit frames -> albedo + surface normals.

I_i = rho * (N . L_i) per light, so stacking all four and solving gives both
rho (true colour, lighting removed - efflorescence) and N (surface normals -
flaking roughness). Needs the LEDs fired ONE AT A TIME; lit together they
give a single combined direction and normals can't be solved at all.

bright_fraction needs a white reference from a calibration target and
returns NaN without one - a percentile threshold would report the same
value for a clean wall and a crusted one, which kills the growth signal.
"""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass, asdict

import numpy as np

from . import config as cfg

# Reflectance above which a patch counts as efflorescence crust. Under
# controlled LED light against a known white target, salt crust sits far above
# fired brick. Calibrate on the validation wall; this is the starting value.
CRUST_REFLECTANCE = 0.62


def light_directions(standoff_mm: float = cfg.STANDOFF_NEAR_MM,
                     radius_mm: float = cfg.LED_RING_RADIUS_MM) -> np.ndarray:
    """Unit vectors from the surface toward each LED.

    Incidence angle is atan(radius / standoff). Around 25-35 deg is the useful
    band: shallower and the normals are poorly conditioned, steeper and the
    surface shadows itself.
    """
    L = []
    for a in cfg.LED_ANGLES_DEG:
        r = math.radians(a)
        v = np.array([radius_mm * math.cos(r), radius_mm * math.sin(r), standoff_mm])
        L.append(v / np.linalg.norm(v))
    return np.array(L)


def incidence_deg(standoff_mm: float = cfg.STANDOFF_NEAR_MM,
                  radius_mm: float = cfg.LED_RING_RADIUS_MM) -> float:
    return math.degrees(math.atan2(radius_mm, standoff_mm))


@dataclass
class SurfaceResult:
    albedo_mean: float          # absolute reflectance if white_ref given, else raw
    bright_fraction: float      # efflorescence: NaN unless a white reference exists
    roughness: float            # flaking: dispersion of surface orientation, deg
    tilt_mean_deg: float
    absolute: bool              # False means bright_fraction is not comparable
    white_ref: float = float("nan")

    def as_dict(self):
        return asdict(self)


def solve(frames: list[np.ndarray], standoff_mm: float = cfg.STANDOFF_NEAR_MM):
    """frames: four single-channel float images, one per LED, same exposure.

    Returns (albedo, normals) with albedo HxW and normals HxWx3.
    """
    if len(frames) != 4:
        raise ValueError("photometric stereo needs exactly 4 frames")
    shp = frames[0].shape
    for f in frames:
        if f.shape != shp:
            raise ValueError("all frames must be the same size")

    L = light_directions(standoff_mm)                              # 4x3
    I = np.stack([f.astype(np.float64).ravel() for f in frames])   # 4xP

    g = np.linalg.lstsq(L, I, rcond=None)[0]                       # 3xP
    rho = np.linalg.norm(g, axis=0)                                # P
    safe = np.where(rho < 1e-9, 1e-9, rho)
    N = (g / safe).T.reshape(shp + (3,))
    return rho.reshape(shp), N


def describe(albedo: np.ndarray, normals: np.ndarray,
             white_ref: float | None = None,
             crust_threshold: float = CRUST_REFLECTANCE) -> SurfaceResult:
    """Collapse the maps into the per-station numbers the risk score uses.
    Without white_ref, bright_fraction comes back NaN rather than a
    non-comparable number."""
    nz = np.clip(normals[..., 2], -1.0, 1.0)
    tilt = np.degrees(np.arccos(np.abs(nz)))
    # dispersion of surface orientation is the roughness signal: a flaking
    # surface scatters its normals, a sound one does not
    roughness = float(np.std(tilt))

    if white_ref is None or not np.isfinite(white_ref) or white_ref <= 0:
        warnings.warn("no white reference: efflorescence fraction is not "
                      "cross-visit comparable and is reported as NaN")
        return SurfaceResult(
            albedo_mean=float(np.mean(albedo)),
            bright_fraction=float("nan"),
            roughness=roughness,
            tilt_mean_deg=float(np.mean(tilt)),
            absolute=False,
        )

    refl = albedo / white_ref            # absolute reflectance units
    return SurfaceResult(
        albedo_mean=float(np.mean(refl)),
        bright_fraction=float(np.mean(refl >= crust_threshold)),
        roughness=roughness,
        tilt_mean_deg=float(np.mean(tilt)),
        absolute=True,
        white_ref=float(white_ref),
    )


def white_reference(albedo: np.ndarray, patch: tuple | None = None) -> float:
    """Median albedo of the white calibration target. patch is
    (row0, row1, col0, col1); the whole frame is used if omitted."""
    if patch:
        r0, r1, c0, c1 = patch
        region = albedo[r0:r1, c0:c1]
    else:
        region = albedo
    return float(np.median(region))


def register(reference: np.ndarray, current: np.ndarray, strict: bool = False):
    """Align this visit's image to the stored reference via feature matching.
    Returns (aligned, (dx, dy), inliers); inliers == 0 means alignment
    failed and the result must not be treated as registered."""
    try:
        import cv2
    except ImportError:
        msg = ("opencv is not installed, so cross-visit registration is "
               "unavailable: pip install opencv-python-headless")
        if strict:
            raise RuntimeError(msg)
        warnings.warn(msg)
        return current, (0.0, 0.0), 0

    def u8(x):
        x = x.astype(np.float64)
        lo, hi = float(np.min(x)), float(np.max(x))
        return ((x - lo) / ((hi - lo) or 1.0) * 255).astype(np.uint8)

    a, b = u8(reference), u8(current)
    orb = cv2.ORB_create(2000)
    ka, da = orb.detectAndCompute(a, None)
    kb, db = orb.detectAndCompute(b, None)
    if da is None or db is None or len(ka) < 12 or len(kb) < 12:
        warnings.warn("too few features to register - is the wall in focus?")
        return current, (0.0, 0.0), 0

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = sorted(matcher.match(db, da), key=lambda m: m.distance)[:200]
    if len(matches) < 12:
        warnings.warn("too few matches to register")
        return current, (0.0, 0.0), 0

    src = np.float32([kb[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    dst = np.float32([ka[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    M, mask = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC,
                                          ransacReprojThreshold=3.0)
    if M is None:
        return current, (0.0, 0.0), 0
    h, w = current.shape[:2]
    aligned = cv2.warpAffine(current, M, (w, h), flags=cv2.INTER_LINEAR)
    return aligned, (float(M[0, 2]), float(M[1, 2])), int(mask.sum())
