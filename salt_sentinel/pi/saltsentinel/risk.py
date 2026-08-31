"""Per-station risk score: w1*moisture_z + w2*efflorescence_growth + w3*flaking_trend.

moisture_z is normalised against the session median, not a fixed threshold,
so a site-wide wet month doesn't read as every station deteriorating at
once. WEIGHTS are provisional until calibrated against the validation wall.
"""

from __future__ import annotations

import statistics
from dataclasses import dataclass

WEIGHTS = {"moisture": 0.60, "efflorescence": 0.25, "flaking": 0.15}
FLAG_THRESHOLD = 0.5


@dataclass
class RiskInputs:
    moisture_index: float               # deg C cooling, negative = wetter
    efflorescence_growth: float = 0.0
    flaking_trend: float = 0.0


def score_session(readings: list[RiskInputs]) -> list[float]:
    """Normalise moisture against this session's own median, then combine."""
    moist = [r.moisture_index for r in readings]
    med = statistics.median(moist) if moist else 0.0
    spread = statistics.pstdev(moist) if len(moist) > 1 else 0.0
    spread = spread or 1.0

    out = []
    for r in readings:
        moist_z = (med - r.moisture_index) / spread   # cooler-than-median -> positive
        s = (WEIGHTS["moisture"] * moist_z
             + WEIGHTS["efflorescence"] * r.efflorescence_growth
             + WEIGHTS["flaking"] * r.flaking_trend)
        out.append(s)
    return out
