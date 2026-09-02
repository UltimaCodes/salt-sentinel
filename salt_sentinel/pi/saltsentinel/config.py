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
I2C_AMG8833 = 0x69     # AD_SELECT -> 3V3

# All VL53L0X ship as 0x29. Reassigned at boot via XSHUT, and the assignment
# is VOLATILE - lost on every power cycle, so the sequence runs every boot.
#
# Placement: wall_a and wall_b are mounted on the CHASSIS, facing the wall,
# side by side - not on the sensor arm, which is now a static bracket
# holding only the thermal array, camera and SHT31. Their mean is still the
# range used for steering (distance + yaw), but since the arm is no longer
# co-located with them, the thermal/camera overlay math (geometry.py) needs
# ARM_TO_CHASSIS_OFFSET_MM added before treating chassis range as arm range.
# 'front' sits on the chassis for corner detection.
#
# wall_b = FRONT (leading, toward the direction of travel), wall_a = BACK.
# This isn't arbitrary - it's forced by the sign convention already baked
# into patrol.py/sensors.py, and swapping the two wires the steering loop
# into a positive-feedback crash instead of a wall-follower:
#   - drive.tank(forward, differential): differential > 0 steers RIGHT.
#   - patrol._steer(): distance > STANDOFF_FAR_MM (too far from the wall)
#     gives differential > 0 -> steers right to close the gap. For that
#     correction to actually work, the wall must be on the robot's RIGHT.
#   - sensors.wall_pose(): yaw = atan2(wall_b - wall_a, baseline), and
#     _steer() adds +YAW_GAIN*yaw to the same right-positive differential.
#     If the FRONT sensor is drifting closer to the (right-side) wall than
#     the back one, the robot needs to steer LEFT (away) to correct - i.e.
#     yaw must go negative in that situation. yaw = atan2(wall_b - wall_a,
#     ...) only goes negative when front-closer iff wall_b IS the front
#     sensor (front closer -> wall_b < wall_a -> wall_b - wall_a < 0).
#     Wire it the other way (wall_a = front) and the same drift makes yaw
#     positive, which *adds* to the turn instead of opposing it - the loop
#     steers harder into the wall the closer it gets.
# Physically: mount wall_b nearer the front of the chassis, wall_a nearer
# the back, both facing the same side as the arm sensors (the wall side).
TOF_FITTED = ("wall_a", "wall_b")          # extend to include "front" when fitted
TOF_XSHUT  = {"wall_a": 5, "wall_b": 6, "front": 13}      # BCM pins
TOF_ADDR   = {"wall_a": 0x30, "wall_b": 0x31, "front": 0x32}
TOF_ORDER  = TOF_FITTED

# ------------------------------------------------------------------ GPIO
# Wall fill light: NOT FITTED YET. This is the planned wiring (one LED ring,
# switched through a single ULN2803A transistor channel on this pin) - there
# is no LED hardware on the rover right now. LED_FITTED gates every caller
# (cli.py's patrol command, and by extension patrol.py's near pass) so
# nothing tries to drive a pin nothing is actually connected to.
LED_FITTED = False
LED_RING_PIN = 17     # BCM

# ------------------------------------------------------------------ geometry
# Spacing of the two chassis-mounted ToF sensors along the direction of
# travel. The yaw calculation scales directly off this - measure it once
# the sensors are actually mounted.
ARM_TOF_BASELINE_MM = 120.0
TOF_BASELINE_MM = ARM_TOF_BASELINE_MM      # backwards-compatible alias

# The ToF pair sits on the chassis; the thermal array/camera sit on the
# static arm out front. Chassis range + this offset = arm-to-wall range,
# which is what geometry.py's overlay math actually needs. 0.0 until the
# arm is built and this is measured - leaving it at 0 silently treats
# chassis range as arm range, which is wrong by exactly the arm's length.
ARM_TO_CHASSIS_OFFSET_MM = 0.0

# Two standoffs, because the channels want opposite distances.
STANDOFF_FAR_MM  = 1000.0   # thermal survey: frame must contain damp AND dry brick
STANDOFF_NEAR_MM = 250.0    # camera detail: micro-texture + usable LED geometry
STANDOFF_TOL_MM  = 15.0

# Only meaningful if a future rev brings the LED ring and photometric
# stereo (see photometric.py) together - neither is fitted right now, this
# is dimensioning math for when they are.
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
