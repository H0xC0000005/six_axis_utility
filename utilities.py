from hyperparams import *
from constants import *

import numpy as np
from scipy.spatial.transform import Rotation as R

def get_field(obj, *fields):
    """
    Traverse nested fields, using dict-access if obj is a dict,
    or getattr otherwise.
    """
    try:
        for fld in fields:
            if isinstance(obj, dict):
                obj = obj[fld]
            else:
                obj = getattr(obj, fld)
    except KeyError or AttributeError as e:
        print(obj)
        raise e
    return obj


def process_carla_imu_message(msg) -> dict[str, float]:
    """
    Convert a carla IMU message, process it, and to a dictionary for six axles.
    This is an adapter that should be called before passing to six axles code.
    """
    g = 9.80665
    get = get_field # alias for simplicity
    result = {}
    result = {}
    # linear_acceleration.x
    result[SURGE_NAME] = get(msg, "linear_acceleration", "x")       # :contentReference[oaicite:5]{index=5}
    result[HEAVE_NAME] = get(msg, "linear_acceleration", "z") - g
    result[SWAY_NAME] = get(msg, "linear_acceleration", "y")

    # orientation → quaternion_to_euler
    q = [
        get(msg, "orientation", "x"),
        get(msg, "orientation", "y"),
        get(msg, "orientation", "z"),
        get(msg, "orientation", "w"),
    ]
    yaw, pitch, roll = quaternion_to_euler(q)
    result[YAW_NAME] = yaw
    result[PITCH_NAME] = pitch
    result[ROLL_NAME] = roll

    # timestamp
    try:
        result[TIMESTAMP_NAME] = get(msg, "header", "stamp", "sec")
    except AttributeError:
        # it is a dict so no sec attr, use its plain timestamp
        result[TIMESTAMP_NAME] = get(msg, "header", "stamp")
    return result


def quaternion_to_euler(quaternion: list | np.ndarray):
    """
    Converts a quaternion to Euler angles (z, y, x).

    Parameters:
    quaternion (numpy.ndarray): A numpy array of shape [4,] or [4, N] where N is the batch size.
                                Each quaternion should be in the order [x, y, z, w].

    Returns:
    numpy.ndarray: A numpy array of shape [3,] or [3, N] representing the Euler angles in radians (z, y, x).
    """
    # Ensure the quaternion is normalized
    quaternion = np.array(quaternion)
    norm = np.linalg.norm(quaternion)
    quaternion = quaternion / norm

    if quaternion.ndim == 1:
        # Single quaternion case, shape [4,]
        r = R.from_quat(
            quaternion
        )  # Assuming input quaternion is in [x, y, z, w] order
        euler = r.as_euler(
            "zyx", degrees=False
        )  # Convert to Euler angles in 'zyx' order
        return euler
    elif quaternion.ndim == 2 and quaternion.shape[0] == 4:
        # Batch of quaternions case, shape [4, N]
        r = R.from_quat(quaternion.T)  # Transpose to shape [N, 4] for scipy
        euler = r.as_euler(
            "zyx", degrees=False
        ).T  # Convert to Euler angles and transpose to [3, N]
        return euler
    else:
        raise ValueError("Input quaternion must have shape [4,] or [4, N]")

    # # Euler angles are returned as (yaw, pitch, roll)
    # yaw, pitch, roll = euler
    # return yaw, pitch, roll
