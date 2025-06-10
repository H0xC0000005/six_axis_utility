import math

"""
six axis platform interface and options
"""

APPLYCOLLISIONNAME = "applyCollision"

YAW_NAME = "Yaw"
PITCH_NAME = "Pitch"
ROLL_NAME = "Roll"
SWAY_NAME = "Sway"
SURGE_NAME = "Surge"
HEAVE_NAME = "Heave"

DIM_NAMES = (YAW_NAME, PITCH_NAME, ROLL_NAME, SWAY_NAME, SURGE_NAME, HEAVE_NAME)

TIMESTAMP_NAME = "Timestamp"

# hardcoded config, according to the manufacturer
# WARNING: DONT CHANGE THIS UNLESS YOU HAVE PRACTICAL REASONS
# SIX_AXIS_RANGES = {
#     ROLL_NAME: 20.0,
#     YAW_NAME: 2.0, 
#     PITCH_NAME: 20.0,
#     SURGE_NAME: 6.0,
#     HEAVE_NAME: 1.0,
#     SWAY_NAME: 4.0,
# }
SIX_AXIS_RANGES = {
    ROLL_NAME: 19.0,
    YAW_NAME: 2.0, 
    PITCH_NAME: 19.0,
    SURGE_NAME: 5.5,
    HEAVE_NAME: 0.95,
    SWAY_NAME: 3.5,
}
# ---------------------------------------------------------------------------
# DATA SOURCE RANGE
# these values are by observation
# and is per-device. denote the device and write this
## carla ----------------------------------------
# IMU_RANGES = {
#     ROLL_NAME: math.pi / 2,
#     YAW_NAME: math.pi,
#     PITCH_NAME: math.pi / 2,
#     SURGE_NAME: 60,
#     HEAVE_NAME: 50,
#     SWAY_NAME: 30,
# }
## evtol ----------------------------------------
# IMU_RANGES = {
#     ROLL_NAME: 90, # orientations in degrees
#     YAW_NAME: 180, # is there a bias? i.e. 0~360?
#     PITCH_NAME: 90,
#     SURGE_NAME: 800, # TODO: currently these three are speeds, in cm/s
#     HEAVE_NAME: 4000, # adjust heave to disable it literally since up and down speeds are great
#     SWAY_NAME: 500,
# }
## wingflight ----------------------------------------
IMU_RANGES = {
    ROLL_NAME: 110, # orientations in degrees
    YAW_NAME: 180, # is there a bias? i.e. 0~360?
    PITCH_NAME: 110,
    SURGE_NAME: 8000, # TODO: currently these three are speeds, in cm/s
    HEAVE_NAME: 3000, # adjust heave to disable it literally since up and down speeds are great
    SWAY_NAME: 2000,
}
# ---------------------------------------------------------------------------


"""
filter related constants. mostly names definition
"""
# whether this dimension is in the output.
ENABLE_DIM_NAME = "enable"

FILTERMODE_ITEM_NAME = "filterMode"
MODE_NAME = "mode"

IDENTITY_MODE_NAME = "identity"

EMA_NAME = "EMA"
ALPHA_NAME = "alpha"

MA_NAME = "MA"

BUTTER_NAME = "Butter"
CUTOFF_NAME = "cutoff"
FS_NAME = "fs"

WIENER_NAME = "Wiener"
NOISE_NAME = "noise"

KALMAN_NAME = "Kalman"
KALMAN_R_NAME = "R"
KALMAN_Q_NAME = "Q"


ORDER_NAME = "order"
AVAILABLE_MODES = {EMA_NAME, MA_NAME, BUTTER_NAME, WIENER_NAME, KALMAN_NAME}

GAIN_NAME = "gain"
# default gain, currently identity. can be modified
DEFAULT_GAIN = 1

COMPRESSOR_NAME = "DynamicRangeCompressor"
COMPRESSOR_THRESHOLD_NAME = "threshold"
# compression ratio. Must > 1; else the program is bugged
COMPRESSOR_RATIO_NAME = "ratio"
COMPRESSOR_ATTACK_NAME = "attack"
COMPRESSOR_RELEASE_NAME = "release"
COMPRESSOR_KNEEWIDTH_NAME = "kneeWidth"
COMPRESSOR_BIAS_NAME = "bias"

CLAMP_THRESHOLD_NAME = "clampThreshold"

"""
misc definitions, like debug variables
"""
DEBUG_PRINT = False
DEBUG = True
