"""Patrol state machine: follow one wall until it ends, sampling as it goes.

Navigation is ToF only: wall_a + wall_b (mean = range, difference = yaw),
front for corner detection. Wall ends when the wall sensors go out of range
or a corner is reached, run() hits max_stations, or the operator hits
Ctrl+C in the terminal running it - that's the manual stop, CLI only,
same as everything else in this project. Ctrl+C still stops the motors and
saves whatever was captured so far, same as a normal end-of-wall.

Two passes per station, opposite standoffs: FAR (~1m) for the thermal
survey (frame needs both damp and dry brick in view), NEAR (~25cm) for
camera detail, only where the far pass actually flagged something.
Stations trigger automatically every STATION_SPACING_MM, or immediately
on demand - pressing space during FOLLOW forces one right away.

On completion, every captured station is re-scored as one session (risk.py,
normalised against this run's own median so a site-wide wet day doesn't
read as universal deterioration), and the same CSV + wall heatmap + PDF
condition report the old synthetic demo pipeline used to produce now get
built from the real data - see _finish(). The run is then saved under a
OUTPUTCLEAN/OUTPUTSALT-suffixed folder (Store.finalize()) and mirrored to
any USB drive that's plugged in.
"""

from __future__ import annotations

import select
import signal
import sys
import time
from enum import Enum, auto
from pathlib import Path

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
        self._any_flagged = False        # -> OUTPUTSALT vs OUTPUTCLEAN at finalize()
        self._photo_paths: list[str] = []  # real wall photos, one per far pass - feeds wallmap

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
        pose = self.hub.wall_pose(speed_mms=0.0)   # stationary, chassis range
        # wall_a/wall_b are on the chassis; the thermal array is out on the
        # static arm, so the optics see a different range than the ToF pair
        # measured - correct it here, once, before anything optical uses it.
        # _steer() elsewhere uses pose.distance_mm directly (uncorrected) on
        # purpose, since steering cares about chassis clearance, not arm range.
        arm_mm = pose.distance_mm + cfg.ARM_TO_CHASSIS_OFFSET_MM
        res = self.thermal.analyse(arm_mm, climate.temp_c)
        rec = StationRecord(
            station=self.station, pass_mode="far",
            standoff_mm=arm_mm, wall_yaw_deg=pose.yaw_deg,
            odo_mm=self.odo_mm,
            air_temp_c=climate.temp_c, rh_pct=climate.rh_pct,
            dew_margin_c=climate.dew_margin_c, raining=climate.raining,
            deliquescence_open=climate.deliquescence_open,
            thermal=res.as_dict(),
            geometry_warnings=geometry.check(arm_mm),
        )
        if not self.thermal.frame_spans_reference(arm_mm):
            rec.notes = ("standoff too close: frame may contain no dry reference, "
                         "differential can read zero on a large damp patch")
        self.store.append(rec)
        self._say(f"station {self.station} far: cooling={res.moisture_index:+.2f}C "
                  f"peak={res.peak_cooling_c:+.2f}C damp_row={res.damp_row} "
                  f"noise={res.noise_floor_c:.3f}C")

        # One real wall photo per station, ambient light (no LED ring fitted
        # yet - see config.LED_FITTED). These feed the end-of-run heatmap
        # panorama in _finish(), in odometry order.
        if self.camera:
            try:
                import cv2
                photo_path = str(self.store.images_dir(self.station) / "wall.jpg")
                cv2.imwrite(photo_path, cv2.cvtColor(self.camera.grab_rgb(), cv2.COLOR_RGB2BGR))
                self._photo_paths.append(photo_path)
            except Exception as e:
                self._say(f"photo capture failed: {e!r}")

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
        arm_mm = pose.distance_mm + cfg.ARM_TO_CHASSIS_OFFSET_MM  # camera is on the arm too
        rec = StationRecord(
            station=self.station, pass_mode="near",
            standoff_mm=arm_mm, wall_yaw_deg=pose.yaw_deg,
            odo_mm=self.odo_mm,
            air_temp_c=climate.temp_c, rh_pct=climate.rh_pct,
            dew_margin_c=climate.dew_margin_c, raining=climate.raining,
            deliquescence_open=climate.deliquescence_open,
            surface=surf,
            geometry_warnings=geometry.check(arm_mm),
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

        term_fd, old_term = None, None
        try:
            import termios, tty
            term_fd = sys.stdin.fileno()
            old_term = termios.tcgetattr(term_fd)
            tty.setcbreak(term_fd)
            self._say("spacebar forces an immediate station capture")
        except Exception:
            self._say("no POSIX terminal - manual station key (space) unavailable")

        try:
            while self.state not in (State.WALL_END, State.FAULT):
                if time.time() - t0 > timeout_s:
                    self._say("timeout")
                    self.state = State.FAULT
                    break

                manual = (term_fd is not None
                          and select.select([sys.stdin], [], [], 0)[0]
                          and sys.stdin.read(1) == " ")

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

                # reached the next station, on distance or the spacebar
                if manual or self.odo_mm - self._last_station_odo >= cfg.STATION_SPACING_MM:
                    self.drive.stop()
                    self._speed_mms = 0.0
                    time.sleep(0.4)
                    self.station += 1
                    self._last_station_odo = self.odo_mm
                    if manual:
                        self._say("manual station trigger (spacebar)")

                    self.state = State.STATION_FAR
                    far = self._capture_far(climate)

                    # Require a genuine spatial gradient (damp_row set - see
                    # thermal.analyse(): the row with the single largest
                    # top-to-bottom step), not just one pixel over threshold.
                    # A real rising-damp line reads as a structured
                    # dry-then-suddenly-cooler transition; a rock, hand or
                    # sheet of plastic held in front of the sensor is close
                    # to isothermal across the whole frame and won't produce
                    # one, so peak_cooling_c alone is not enough to flag on -
                    # standard practice in building IRT is exactly this:
                    # gradient/anomaly shape, not a single reading, is what
                    # separates a real defect from sensor noise or clutter.
                    peak_hit = far.thermal.get("peak_cooling_c", 0.0) >= self.flag_threshold_c
                    gradient_hit = far.thermal.get("damp_row", -1) > 0
                    flagged = peak_hit and gradient_hit
                    self._any_flagged = self._any_flagged or flagged
                    if flagged and self.camera and self.leds:
                        self.state = State.STATION_NEAR
                        self._say("flagged - running near pass")
                        self._capture_near(climate)
                    elif flagged:
                        self._say("flagged, but no camera/LED ring fitted - skipping near pass")
                    elif peak_hit:
                        self._say("cooling seen but no coherent gradient - not flagging "
                                  "(likely not a wall surface)")
                    else:
                        self._say("no cooling above threshold, skipping near pass")

                    if self.station >= max_stations:
                        self.state = State.WALL_END
                        break
                    self.state = State.FOLLOW
                    continue

                # normal wall following
                self.drive.tank(cfg.CRUISE, self._steer(pose))
                self._speed_mms = cfg.CRUISE * 0.9      # rough, no encoders to refine it
                self.odo_mm += self._speed_mms * tick_s
                time.sleep(tick_s)
        except KeyboardInterrupt:
            self._say("Ctrl+C - stopping and saving what was captured so far")
        finally:
            self.drive.stop()
            self.drive.enable(False)
            if term_fd is not None:
                termios.tcsetattr(term_fd, termios.TCSADRAIN, old_term)

        self._say(f"finished in state {self.state.name} after {self.station} stations")
        self._say("saving - a second Ctrl+C here is ignored until this finishes")
        old_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            self._finish()
        finally:
            signal.signal(signal.SIGINT, old_sigint)
        return self.state

    def _finish(self) -> Path:
        """Score the session, write the CSV/heatmap/PDF report, then
        finalize (rename + USB mirror) last so the mirror picks up
        everything - jsonl, csv, heatmap, pdf - in one pass."""
        from . import risk as risk_mod
        from . import wallmap, report_pdf

        records = self.store.load()
        if records:
            inputs = [risk_mod.RiskInputs(
                moisture_index=(r.get("thermal") or {}).get("moisture_index", 0.0) or 0.0,
                efflorescence_growth=(r.get("surface") or {}).get("bright_fraction", 0.0) or 0.0,
                flaking_trend=((r.get("surface") or {}).get("roughness", 0.0) or 0.0) / 10.0,
            ) for r in records]
            for rec, score in zip(records, risk_mod.score_session(inputs)):
                rec["risk_score"] = score
                rec["flagged"] = score >= risk_mod.FLAG_THRESHOLD
            self.store.rewrite(records)

        csv_path = self.store.export_csv()
        self._say(f"csv: {csv_path}")

        heatmap_path = ""
        if self._photo_paths and records:
            try:
                res = wallmap.render(
                    self._photo_paths,
                    station_odo_mm=[r.get("odo_mm") or 0.0 for r in records],
                    station_cooling_c=[(r.get("thermal") or {}).get("moisture_index", 0.0) or 0.0
                                       for r in records],
                    station_damp_height_mm=[(r.get("thermal") or {}).get("damp_height_mm", 0.0) or 0.0
                                            for r in records],
                    out_path=str(self.store.dir / "wall_heatmap.png"),
                )
                heatmap_path = res.out_path
                self._say(f"heatmap ({res.stitched_from} photo"
                          f"{'s' if res.stitched_from != 1 else ''}): {heatmap_path}")
            except Exception as e:
                self._say(f"heatmap generation failed: {e!r}")
        else:
            self._say("no photos captured (camera not fitted or --no-camera) - skipping heatmap")

        if records:
            pdf_path = self.store.dir / "condition_report.pdf"
            try:
                report_pdf.build(records, heatmap_path, str(pdf_path))
                self._say(f"pdf: {pdf_path}")
            except Exception as e:
                self._say(f"pdf generation failed: {e!r}")

        final_dir = self.store.finalize(self._any_flagged)
        self._say(f"saved to {final_dir} ({'OUTPUTSALT' if self._any_flagged else 'OUTPUTCLEAN'}, "
                  f"+ USB copy if a drive was plugged in)")
        return final_dir
