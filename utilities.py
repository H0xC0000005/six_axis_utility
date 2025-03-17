from hyperparams import *
from constants import *

import numpy as np
from scipy.spatial.transform import Rotation as R


def convert_carla_imu_message_to_dict(msg) -> dict[str, float]:
    """
    Convert a carla IMU message to a dictionary for six axles.
    This is an adapter that should be called before passing to six axles code.
    """
    g = 9.80665
    try:
        result = {}
        result[SURGE_NAME] = msg.linear_acceleration.x
        result[HEAVE_NAME] = msg.linear_acceleration.z - g
        result[SWAY_NAME] = msg.linear_acceleration.y

        q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]
        yaw, pitch, roll = quaternion_to_euler(q)
        result[YAW_NAME] = yaw
        result[PITCH_NAME] = pitch
        result[ROLL_NAME] = roll
        # ROS1
        # result[TIMESTAMP_NAME] = msg.header.stamp.to_sec()
        result[TIMESTAMP_NAME] = msg.header.stamp.sec
        return result
    except ValueError as e:
        print(e)
        raise ValueError(
            "provided msg cannot be converted to dictionary based on IMU protocol of carla."
        )


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
