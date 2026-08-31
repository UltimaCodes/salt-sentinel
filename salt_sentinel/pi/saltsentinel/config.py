"""Central configuration. Every tunable the rest of the code reads lives here."""

from dataclasses import dataclass, field
from pathlib import Path

# ------------------------------------------------------------------ paths
ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data"
CALIB = DATA / "calib"

# ------------------------------------------------------------------ I2C map
# Addresses are set deliberately, never inherited from module defaults.
I2C_SHT31   = 0x44
I2C_MLX     = 0x5A
I2C_INA219  = 0x40
I2C_MPU6050 = 0x68     # AD0 -> GND
I2C_AMG8833 = 0x69     # AD_SELECT -> 3V3

# All VL53L0X ship as 0x29. Reassigned at boot via XSHUT, and the assignment
# is VOLATILE - lost on every power cycle, so the sequence runs every boot.
#
# Recommended placement: wall_a and wall_b side by side ON THE SENSOR ARM,
# boresighted with the camera and thermal array. Their mean is the range used
# for every scale and overlay calculation; their difference is the arm's yaw to
# the wall. Because the arm is rigid to the chassis while following, the same
# pair also steers. 'front' sits on the chassis for corner detection.
TOF_FITTED = ("wall_a", "wall_b")          # extend to include "front" when fitted
TOF_XSHUT  = {"wall_a": 5, "wall_b": 6, "front": 13}      # BCM pins
TOF_ADDR   = {"wall_a": 0x30, "wall_b": 0x31, "front": 0x32}
TOF_ORDER  = TOF_FITTED

# ------------------------------------------------------------------ GPIO
# Wall fill light: one LED ring, switched through a single transistor.
LED_RING_PIN = 17     # BCM

# ------------------------------------------------------------------ geometry
# Spacing of the two arm-mounted ToF sensors along the direction of travel.
# Measure this once the arm is built - the yaw calculation scales directly
# off it.
ARM_TOF_BASELINE_MM = 120.0
TOF_BASELINE_MM = ARM_TOF_BASELINE_MM      # backwards-compatible alias
ARM_TO_CHASSIS_OFFSET_MM = 0.0             # measure once the arm is built

# Two standoffs, because the channels want opposite distances.
STANDOFF_FAR_MM  = 1000.0   # thermal survey: frame must contain damp AND dry brick
STANDOFF_NEAR_MM = 250.0    # camera detail: micro-texture + usable LED geometry
STANDOFF_TOL_MM  = 15.0

# Only meaningful if a future rev brings back multi-light photometric
# stereo (see photometric.py) - the current single-ring fill light doesn't
# use these.
LED_RING_RADIUS_MM = 120.0
LED_ANGLES_DEG = (0.0, 90.0, 180.0, 270.0)

# ---- sensor-head optics -------------------------------------------------
# Physical offset between the AMG8833 centre and the camera lens centre. This
# is what makes the overlay distance-dependent: the offset is fixed in mm but
# scales in pixels, so an assumed range puts the two frames on different bricks.
THERMAL_CAM_BASELINE_MM = 40.0
CAM_FOV_DEG = 53.5          # OV5647 horizontal; measure yours and correct this
CAM_PX_WIDTH = 2028
CAM_PX_HEIGHT = 1520

# ------------------------------------------------------------------ thermal
AMG_FOV_DEG = 60.0
AMG_ROWS = AMG_COLS = 8
# Rows used as the in-frame dry reference. The head is oriented so the top of
# the frame sits above the rising-damp line.
DRY_REFERENCE_ROWS = 3
# Frames averaged per station. Noise falls as sqrt(N) and the rover is
# stationary anyway, so this is close to free.
THERMAL_FRAMES = 30
THERMAL_FRAME_DELAY_S = 0.11    # AMG8833 runs at 10 Hz

# ------------------------------------------------------------------ drive
SERIAL_PORT = "/dev/ttyUSB0"    # prefer /dev/serial/by-id/... - it never renumbers
SERIAL_BAUD = 115200
# Must be comfortably under the firmware's 500 ms watchdog.
DRIVE_KEEPALIVE_S = 0.15

CRUISE = 320            # -1000..1000
TURN_GAIN = 2.2         # standoff error (mm) -> differential
YAW_GAIN = 9.0          # wall yaw (deg) -> differential
MAX_DIFF = 340

# ------------------------------------------------------------------ patrol
WALL_LOST_MM = 1400     # wall sensors beyond this: the wall has ended
FRONT_STOP_MM = 300     # front sensor closer than this: corner reached
STATION_SPACING_MM = 400


@dataclass
class Climate:
    """One ambient sample. Every reading is tagged with this - a measurement
    taken under non-comparable conditions is not comparable."""
    temp_c: float = float("nan")
    rh_pct: float = float("nan")
    dew_c: float = float("nan")
    dew_margin_c: float = float("nan")
    raining: bool = False

    @property
    def deliquescence_open(self) -> bool:
        """The seasonal salt-confirmation channel is available only inside a
        band: high enough RH for salts to take up water, but far enough from
        dew point that plain condensation is not the explanation."""
        return 60.0 <= self.rh_pct <= 92.0 and self.dew_margin_c >= 2.0


@dataclass
class Station:
    index: int
    odo_mm: float = 0.0
    heights: tuple = field(default_factory=lambda: ("low", "mid"))
