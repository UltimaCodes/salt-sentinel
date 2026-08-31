"""Patrol state machine: follow one wall until it ends, sampling as it goes.

Navigation is ToF only: wall_a + wall_b (mean = range, difference = yaw),
front for corner detection. Wall ends when the wall sensors go out of range.

Two passes per station, opposite standoffs: FAR (~1m) for the thermal
survey (frame needs both damp and dry brick in view), NEAR (~25cm) for
camera detail, only where the far pass actually flagged something.
"""

from __future__ import annotations

import time
from enum import Enum, auto

from . import config as cfg
from .store import StationRecord, Store


class State(Enum):
    IDLE = auto()
    FOLLOW = auto()
    STATION_FAR = auto()
    STATION_NEAR = auto()
    CORNER = auto()
    WALL_END = auto()
    FAULT = auto()


class Patrol:
    def __init__(self, drive, hub, thermal, camera=None, leds=None,
                 store: Store | None = None, flag_threshold_c: float = 0.6):
        self.drive = drive
        self.hub = hub
        self.thermal = thermal
        self.camera = camera
        self.leds = leds
        self.store = store or Store()
        self.state = State.IDLE
        self.station = 0
        self.odo_mm = 0.0
        self.flag_threshold_c = flag_threshold_c
        self.white_ref = None      # set by calibrate_white() before the pass
        self.raining = False       # operator-set: no rain sensor in rev 2
        self._last_station_odo = 0.0
        self._speed_mms = 0.0      # feeds the single-sensor yaw fallback
        self._enc0 = None
        self.log: list[str] = []

    # ------------------------------------------------------------- utilities
    def _say(self, msg: str):
        line = f"[{time.strftime('%H:%M:%S')}] {self.state.name:<12} {msg}"
        self.log.append(line)
        print(line)

    def _steer(self, pose):
        """Hold standoff and stay parallel. Two terms, two sensors."""
        err = pose.distance_mm - cfg.STANDOFF_FAR_MM
        diff = cfg.TURN_GAIN * err
        if pose.yaw_deg == pose.yaw_deg:      # not NaN
            diff += cfg.YAW_GAIN * pose.yaw_deg
        return max(-cfg.MAX_DIFF, min(cfg.MAX_DIFF, diff))

    def calibrate_white(self):
        """Image the white calibration card before the pass, or the
        efflorescence channel reports NaN instead of a comparable number."""
        if not (self.camera and self.leds):
            return None
        import numpy as np
        with self.leds.lit():
            frame = self.camera.grab_gray()
        self.white_ref = float(np.median(frame))
        self._say(f"white reference = {self.white_ref:.1f}")
        return self.white_ref

    # ------------------------------------------------------------- stations
    def _capture_far(self, climate) -> StationRecord:
        from . import geometry
        pose = self.hub.wall_pose(speed_mms=0.0)   # stationary
        res = self.thermal.analyse(pose.distance_mm, climate.temp_c)
        rec = StationRecord(
            station=self.station, pass_mode="far",
            standoff_mm=pose.distance_mm, wall_yaw_deg=pose.yaw_deg,
            odo_mm=self.odo_mm,
            air_temp_c=climate.temp_c, rh_pct=climate.rh_pct,
            dew_margin_c=climate.dew_margin_c, raining=climate.raining,
            deliquescence_open=climate.deliquescence_open,
            thermal=res.as_dict(),
            surface_temp_c=self.hub.surface_temp_c(),
            geometry_warnings=geometry.check(pose.distance_mm),
        )
        if not self.thermal.frame_spans_reference(pose.distance_mm):
            rec.notes = ("standoff too close: frame may contain no dry reference, "
                         "differential can read zero on a large damp patch")
        self.store.append(rec)
        self._say(f"station {self.station} far: cooling={res.moisture_index:+.2f}C "
                  f"peak={res.peak_cooling_c:+.2f}C damp_row={res.damp_row} "
                  f"noise={res.noise_floor_c:.3f}C")
        return rec

    def _capture_near(self, climate) -> StationRecord | None:
        """Efflorescence detail under the fill ring. Only called when the far
        pass flagged this spot. One light source measures reflectance
        against the white reference; it can't recover surface roughness/
        flaking, which needs directional lighting this rig doesn't have."""
        if not (self.camera and self.leds):
            return None
        import numpy as np
        from . import geometry
        from .photometric import CRUST_REFLECTANCE

        with self.leds.lit():
            frame = self.camera.grab_gray()

        if self.white_ref and self.white_ref > 0:
            refl = frame / self.white_ref
            surf = {
                "albedo_mean": float(np.mean(refl)),
                "bright_fraction": float(np.mean(refl >= CRUST_REFLECTANCE)),
                "roughness": float("nan"),
                "tilt_mean_deg": float("nan"),
                "absolute": True,
                "white_ref": float(self.white_ref),
            }
        else:
            surf = {
                "albedo_mean": float(np.mean(frame)),
                "bright_fraction": float("nan"),
                "roughness": float("nan"),
                "tilt_mean_deg": float("nan"),
                "absolute": False,
                "white_ref": float("nan"),
            }

        pose = self.hub.wall_pose(speed_mms=0.0)
        rec = StationRecord(
            station=self.station, pass_mode="near",
            standoff_mm=pose.distance_mm, wall_yaw_deg=pose.yaw_deg,
            odo_mm=self.odo_mm,
            air_temp_c=climate.temp_c, rh_pct=climate.rh_pct,
            dew_margin_c=climate.dew_margin_c, raining=climate.raining,
            deliquescence_open=climate.deliquescence_open,
            surface=surf,
            geometry_warnings=geometry.check(pose.distance_mm),
        )
        self.store.append(rec)
        bright = surf["bright_fraction"]
        self._say(f"station {self.station} near: bright="
                  f"{'n/a' if bright != bright else f'{bright:.3f}'}")
        return rec

    # ------------------------------------------------------------------ run
    def run(self, max_stations: int = 40, tick_s: float = 0.05,
            timeout_s: float = 600.0):
        self.state = State.FOLLOW
        self.drive.enable(True)
        t0 = time.time()

        try:
            while self.state not in (State.WALL_END, State.FAULT):
                if time.time() - t0 > timeout_s:
                    self._say("timeout")
                    self.state = State.FAULT
                    break

                pose = self.hub.wall_pose(speed_mms=self._speed_mms)
                front = self.hub.read_front()
                # No rain sensor on the ESP32 in rev 2. Set .raining from
                # outside, otherwise the monsoon ambiguity flag is never raised.
                climate = self.hub.climate(raining=self.raining)

                # wall ended
                if pose.distance_mm != pose.distance_mm or pose.distance_mm > cfg.WALL_LOST_MM:
                    self.drive.stop()
                    self.state = State.WALL_END
                    self._say("wall sensors out of range - wall complete")
                    break

                # corner: something perpendicular ahead. NaN means the front
                # sensor is not fitted, which is 'no information', not 'clear'.
                if front == front and 0 < front < cfg.FRONT_STOP_MM:
                    self.drive.stop()
                    self.state = State.CORNER
                    self._say(f"corner at {front:.0f} mm")
                    break

                # reached the next station
                if self.odo_mm - self._last_station_odo >= cfg.STATION_SPACING_MM:
                    self.drive.stop()
                    self._speed_mms = 0.0
                    time.sleep(0.4)
                    self.station += 1
                    self._last_station_odo = self.odo_mm

                    self.state = State.STATION_FAR
                    far = self._capture_far(climate)

                    flagged = far.thermal.get("peak_cooling_c", 0.0) >= self.flag_threshold_c
                    if flagged:
                        self.state = State.STATION_NEAR
                        self._say("flagged - running near pass")
                        self._capture_near(climate)
                    else:
                        self._say("no cooling above threshold, skipping near pass")

                    if self.station >= max_stations:
                        self.state = State.WALL_END
                        break
                    self.state = State.FOLLOW
                    continue

                # normal wall following
                self.drive.tank(cfg.CRUISE, self._steer(pose))
                self._speed_mms = cfg.CRUISE * 0.9      # rough, refined by encoders
                self.odo_mm += self._speed_mms * tick_s
                time.sleep(tick_s)
        finally:
            self.drive.stop()
            self.drive.enable(False)

        self._say(f"finished in state {self.state.name} after {self.station} stations")
        return self.state
