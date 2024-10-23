from hyperparams import *
from constants import *
import numpy as np
from scipy.spatial.transform import Rotation as R


def convert_carla_imu_message_to_dict(msg):
    g = 9.80665
    try:
        result = {}
        result[SURGE_NAME] = msg.linear_acceleration.x
        result[HEAVE_NAME] = msg.linear_acceleration.z - g
        result[SWAY_NAME] = msg.linear_acceleration.y
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        yaw, pitch, roll = quaternion_to_euler(q)
        result[YAW_NAME] = yaw
        result[PITCH_NAME] = pitch
        result[ROLL_NAME] = roll
        result[TIMESTAMP_NAME] = msg.header.stamp.to_sec()
        return result
    except ValueError | Exception as e:
        print(e)
        raise ValueError(
            "provided msg cannot be converted to dictionary based on IMU protocol of carla."
        )


def quaternion_to_euler(quaternion):
    quaternion = np.array(quaternion)
    norm = np.linalg.norm(quaternion)
    quaternion = quaternion / norm
    if quaternion.ndim == 1:
        r = R.from_quat(quaternion)
        euler = r.as_euler("zyx", degrees=False)
        return euler
    elif quaternion.ndim == 2 and quaternion.shape[0] == 4:
        r = R.from_quat(quaternion.T)
        euler = r.as_euler("zyx", degrees=False).T
        return euler
    else:
        raise ValueError("Input quaternion must have shape [4,] or [4, N]")
