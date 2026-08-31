"""I2C sensors: ToF ranging, climate, IMU, current monitor.

Every VL53L0X ships at 0x29, so they're woken one at a time via XSHUT and
reassigned - the new addresses are RAM-only, so this has to run every boot.
The two wall-facing units belong on the sensor arm (not the chassis),
boresighted with the camera/thermal array, since their range sets the
physical scale for every measurement and overlay.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from . import config as cfg

try:
    import board
    import busio
    import adafruit_vl53l0x
    import adafruit_sht31d
    import adafruit_mpu6050
    import adafruit_ina219
    from gpiozero import DigitalOutputDevice
    HW = True
    HW_ERR = None
except Exception as e:  # pragma: no cover - laptop development
    HW = False
    HW_ERR = e

try:
    import adafruit_mlx90614
    HAS_MLX = True
except Exception:
    HAS_MLX = False


def dew_point_c(temp_c: float, rh_pct: float) -> float:
    """Magnus-Tetens. Gates the deliquescence channel and rejects readings
    taken too close to condensation."""
    rh = max(1.0, min(100.0, rh_pct))
    a, b = 17.62, 243.12
    g = math.log(rh / 100.0) + (a * temp_c) / (b + temp_c)
    return (b * g) / (a - g)


@dataclass
class WallPose:
    """Distance alone isn't enough - the same standoff at a different angle
    is a different measurement, so yaw comes along with it."""
    distance_mm: float
    yaw_deg: float
    valid: bool
    n_sensors: int = 0


class SensorHub:
    def __init__(self, simulate: bool = False):
        if not simulate and not HW:
            raise RuntimeError(
                "real hardware requested (no --sim) but the sensor driver "
                "libraries are not importable, so this would otherwise have "
                "SILENTLY returned simulated numbers instead of failing. "
                f"Import error: {HW_ERR!r}. Run pip install -r requirements.txt "
                "inside the venv (see PI_SETUP.md), or pass --sim if you meant "
                "to run without hardware.")
        self.simulate = simulate
        self.tof = {}
        self._xshut = {}
        self._prev = None
        self.sht = None
        self.imu = None
        self.ina = None
        self.mlx = None
        if self.simulate:
            return

        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100_000)
        self._init_tof()
        self.sht = adafruit_sht31d.SHT31D(self.i2c, address=cfg.I2C_SHT31)
        self.imu = adafruit_mpu6050.MPU6050(self.i2c, address=cfg.I2C_MPU6050)
        self.ina = adafruit_ina219.INA219(self.i2c, addr=cfg.I2C_INA219)
        if HAS_MLX:
            try:
                self.mlx = adafruit_mlx90614.MLX90614(self.i2c, address=cfg.I2C_MLX)
            except Exception:
                self.mlx = None

    # ------------------------------------------------------------------ ToF
    def _init_tof(self):
        """Wake each VL53L0X alone and give it a unique address. Must run
        every boot - if i2cdetect still shows 0x29 after, this didn't run."""
        for name in cfg.TOF_FITTED:
            self._xshut[name] = DigitalOutputDevice(cfg.TOF_XSHUT[name],
                                                    initial_value=False)
        time.sleep(0.05)                      # everyone held in reset

        for name in cfg.TOF_FITTED:
            self._xshut[name].on()            # wake exactly one
            time.sleep(0.05)
            sensor = adafruit_vl53l0x.VL53L0X(self.i2c)   # answers at 0x29
            sensor.set_address(cfg.TOF_ADDR[name])
            sensor.measurement_timing_budget = 50000      # 50 ms, less noise
            self.tof[name] = sensor

    def read_tof(self, name: str) -> float:
        if self.simulate:
            return {"wall_a": 248.0, "wall_b": 252.0, "front": 1800.0}.get(name, float("nan"))
        if name not in self.tof:
            return float("nan")
        try:
            return float(self.tof[name].range)
        except Exception:
            return float("nan")

    def read_front(self) -> float:
        """Forward range for corner detection. Returns NaN if not fitted, which
        the patrol treats as 'no corner information' rather than 'no corner'."""
        if "front" not in cfg.TOF_FITTED:
            return float("nan")
        return self.read_tof("front")

    def wall_pose(self, speed_mms: float = 0.0) -> WallPose:
        """Range and yaw to the wall - direct with both arm sensors fitted,
        estimated from range-rate (noisier, needs motion) with only one."""
        have = [n for n in ("wall_a", "wall_b") if n in cfg.TOF_FITTED]
        if len(have) >= 2:
            a, b = self.read_tof("wall_a"), self.read_tof("wall_b")
            if not (math.isfinite(a) and math.isfinite(b)):
                return WallPose(float("nan"), float("nan"), False, 2)
            if a <= 0 or b <= 0 or a > 2000 or b > 2000:
                return WallPose((a + b) / 2.0, float("nan"), False, 2)
            yaw = math.degrees(math.atan2(b - a, cfg.ARM_TOF_BASELINE_MM))
            return WallPose((a + b) / 2.0, yaw, True, 2)
        if len(have) == 1:
            return self._wall_pose_single(have[0], speed_mms)
        return WallPose(float("nan"), float("nan"), False, 0)

    def _wall_pose_single(self, name: str, speed_mms: float) -> WallPose:
        """One-sensor fallback: d(range)/d(travel) = sin(yaw). Only valid
        while moving, and can't tell a sloping wall from a yawed rover."""
        d = self.read_tof(name)
        now = time.time()
        yaw = float("nan")
        if math.isfinite(d) and 0 < d <= 2000:
            if self._prev is not None and speed_mms > 20.0:
                pd, pt = self._prev
                dt = now - pt
                if dt > 0.02:
                    ddot = (d - pd) / dt
                    yaw = math.degrees(math.asin(max(-1.0, min(1.0, ddot / speed_mms))))
            self._prev = (d, now)
            return WallPose(d, yaw, True, 1)
        return WallPose(d, float("nan"), False, 1)

    # -------------------------------------------------------------- climate
    def climate(self, raining: bool = False) -> cfg.Climate:
        if self.simulate:
            t, rh = 28.0, 72.0
        else:
            try:
                t = float(self.sht.temperature)
                rh = float(self.sht.relative_humidity)
            except Exception:
                return cfg.Climate(raining=raining)
        dew = dew_point_c(t, rh)
        return cfg.Climate(temp_c=t, rh_pct=rh, dew_c=dew,
                           dew_margin_c=t - dew, raining=raining)

    def surface_temp_c(self) -> float:
        """Absolute surface temperature (MLX90614), anchoring the AMG8833
        which is accurate pixel-to-pixel but not in absolute terms."""
        if self.simulate:
            return 26.4
        if not self.mlx:
            return float("nan")
        try:
            return float(self.mlx.object_temperature)
        except Exception:
            return float("nan")

    # ------------------------------------------------------------------ misc
    def heading_rate(self) -> float:
        if self.simulate:
            return 0.0
        try:
            return float(self.imu.gyro[2])
        except Exception:
            return 0.0

    def rail_current_ma(self) -> float:
        if self.simulate:
            return 480.0
        try:
            return float(self.ina.current)
        except Exception:
            return float("nan")

    def close(self):
        for d in self._xshut.values():
            try:
                d.close()
            except Exception:
                pass
