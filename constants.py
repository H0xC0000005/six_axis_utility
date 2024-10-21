"""
name of each axis for six axis platform
"""

YAW_NAME = "Yaw"
PITCH_NAME = "Pitch"
ROLL_NAME = "Roll"
SWAY_NAME = "Sway"
SURGE_NAME = "Surge"
HEAVE_NAME = "Heave"

TIMESTAMP_NAME = "Timestamp"

"""
filter related constants. mostly names definition
"""
# whether this dimension is in the output.
ENABLE_DIM_NAME = "enable"

FILTERMODE_ITEM_NAME = "FilterMode"
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

"""
misc definitions, like debug variables
"""
DEBUG_PRINT = False
DEBUG = True
