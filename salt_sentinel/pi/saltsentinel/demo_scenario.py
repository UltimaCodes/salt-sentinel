"""Simulated demo scenario - a staged patrol of a makeshift wall (some
bricks soaked in salt water, some left dry), run through the same
StationRecord / risk-score / CSV / PDF pipeline real field data will use.
"""

from __future__ import annotations

import random
import time

from . import config as cfg
from .store import Store, StationRecord
from .risk import RiskInputs, score_session, FLAG_THRESHOLD

# True = this brick was soaked in saline water before the wall was built.
STATION_SALTED = [False, False, True, True, False, True, False, False, True, False]


def _thermal_dict(salted: bool, rng: random.Random) -> dict:
    """Same shape as ThermalCamera.analyse()'s ThermalResult.as_dict()."""
    dry = 27.0 + rng.uniform(-0.3, 0.3)
    if salted:
        cooling = rng.uniform(1.4, 2.3)     # evaporative cooling, salted+wet brick
        peak = cooling + rng.uniform(0.2, 0.6)
        damp_row = rng.choice([4, 5, 6])
        damp_h = (cfg.AMG_ROWS - damp_row) / cfg.AMG_ROWS * 580.0
    else:
        cooling = rng.uniform(-0.15, 0.15)  # dry control brick: no signal
        peak = cooling + rng.uniform(0.0, 0.2)
        damp_row = -1
        damp_h = 0.0
    return {
        "dry_reference_c": dry,
        "moisture_index": -cooling,          # negative = cooler = wetter, matches thermal.py
        "peak_cooling_c": -peak,
        "damp_row": damp_row,
        "damp_height_mm": damp_h,
        "below_ambient": salted,
        "noise_floor_c": rng.uniform(0.008, 0.015),
        "mm_per_px": 72.5,     # frame_width_mm(1000mm) / 8, matches STANDOFF_FAR_MM
        "stamp": time.time(),
    }


def _surface_dict(salted: bool, rng: random.Random) -> dict:
    """Same shape as photometric.SurfaceResult.as_dict()."""
    if salted:
        bright = rng.uniform(0.18, 0.34)     # early efflorescence, not full crust yet
        rough = rng.uniform(4.5, 7.0)
    else:
        bright = rng.uniform(0.0, 0.05)
        rough = rng.uniform(1.0, 2.2)
    return {
        "albedo_mean": bright * 1.4,
        "bright_fraction": bright,
        "roughness": rough,
        "tilt_mean_deg": rng.uniform(2.0, 5.0),
        "absolute": True,
        "white_ref": 0.91,
    }


def build_run(seed: int = 20260825) -> Store:
    """Write one full synthetic patrol to a fresh Store and return it."""
    rng = random.Random(seed)
    store = Store(run_name=f"DEMO_run_{time.strftime('%Y%m%d_%H%M%S')}")

    climate = dict(air_temp_c=34.5, rh_pct=41.0, dew_margin_c=9.8, raining=False)

    records: list[StationRecord] = []
    for i, salted in enumerate(STATION_SALTED):
        th = _thermal_dict(salted, rng)
        flagged_far = th["peak_cooling_c"] <= -0.6     # matches Patrol.flag_threshold_c
        rec = StationRecord(
            station=i + 1, pass_mode="far",
            standoff_mm=1000.0 + rng.uniform(-20, 20),
            wall_yaw_deg=rng.uniform(-2.0, 2.0),
            odo_mm=i * cfg.STATION_SPACING_MM,
            thermal=th,
            surface=_surface_dict(salted, rng) if flagged_far else {},
            **climate,
            deliquescence_open=(60.0 <= climate["rh_pct"] <= 92.0
                                and climate["dew_margin_c"] >= 2.0),
            notes="SIMULATED - staged demo scenario, not a real reading",
        )
        records.append(rec)

    scored = score_session([
        RiskInputs(moisture_index=r.thermal["moisture_index"],
                   efflorescence_growth=r.surface.get("bright_fraction", 0.0),
                   flaking_trend=r.surface.get("roughness", 0.0) / 10.0)
        for r in records
    ])
    for rec, s in zip(records, scored):
        rec.risk_score = s
        rec.flagged = s >= FLAG_THRESHOLD
        store.append(rec)

    return store
