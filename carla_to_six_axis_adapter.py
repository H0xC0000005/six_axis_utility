from typing import Iterable, Sequence, Sized

try:
    import rospy
    from sensor_msgs.msg import Imu
except ImportError:
    print(f"!!!ARNING: rospy or sensor_msgs not available.")
import time
import math
import socket
from control_sender import CustomGamePack, ControlSender, GameStatus, send_pack
from hyperparams import *
from utilities import *
from equalizer import Equalizer
from recorder import Recorder


class IMUMotionController:

    def __init__(self, equalizer=None, recorder_imu=None, recorder_output=None):
        rospy.init_node("imu_motion_controller")
        self.subscription = rospy.Subscriber("/imu/imu", Imu, self.imu_callback)
        self.control_sender = ControlSender()
        ip_address = "192.168.0.208"
        port = 15620
        self.end_point = ip_address, port
        self.udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.equalizer = equalizer
        self.do_equalization = True
        self.recorder_imu = recorder_imu
        self.recorder_output = recorder_output

    def imu_callback(self, msg):
        result_dict = convert_carla_imu_message_to_dict(msg)
        if self.recorder_imu is not None:
            self.recorder_imu.update_record(result_dict)
        self.pack_imu_to_six_axis_value(result_dict)
        if self.do_equalization and self.equalizer is not None:
            result_dict = self.equalizer.equalize_pipeline(result_dict)
        result = CustomGamePack.from_dict(result_dict)
        result.GameStatus = GameStatus.GameStart
        if self.recorder_output is not None:
            self.recorder_output.update_record(result_dict)
        send_pack(self.udp_client, self.end_point, result)

    def run(self):
        rospy.spin()

    def stop(self):
        if self.recorder_imu is not None:
            self.recorder_imu.to_csv(f"./carla_data/imu_{time.time()}.csv")
        if self.recorder_output is not None:
            self.recorder_output.to_csv(f"./carla_data/output_{time.time()}.csv")

    def multiply_imu(self, pack_dict, multipliers=(1, 1)):
        if not isinstance(multipliers, Sequence):
            position_multiplier, acceleration_multiplier = (multipliers, multipliers)
        else:
            position_multiplier, acceleration_multiplier = (
                multipliers[0],
                multipliers[1],
            )
        pack_dict[SURGE_NAME], pack_dict[HEAVE_NAME], pack_dict[SWAY_NAME] = (
            pack_dict[SURGE_NAME] * acceleration_multiplier,
            pack_dict[HEAVE_NAME] * acceleration_multiplier,
            pack_dict[SWAY_NAME] * acceleration_multiplier,
        )
        pack_dict[YAW_NAME], pack_dict[PITCH_NAME], pack_dict[ROLL_NAME] = (
            pack_dict[YAW_NAME] * position_multiplier,
            pack_dict[PITCH_NAME] * position_multiplier,
            pack_dict[ROLL_NAME] * position_multiplier,
        )

    @staticmethod
    def imu_to_six_axis_value(dim_name, reading):
        ranges = {
            ROLL_NAME: 20.0,
            YAW_NAME: 2.0,
            PITCH_NAME: 20.0,
            SURGE_NAME: 6.0,
            HEAVE_NAME: 1.0,
            SWAY_NAME: 4.0,
        }
        imu_ranges = {
            ROLL_NAME: math.pi / 2,
            YAW_NAME: math.pi,
            PITCH_NAME: math.pi / 2,
            SURGE_NAME: 60,
            HEAVE_NAME: 50,
            SWAY_NAME: 30,
        }
        sign = 1
        if dim_name in (PITCH_NAME, ROLL_NAME):
            sign = -1
        return reading / imu_ranges[dim_name] * ranges[dim_name] * sign

    def pack_imu_to_six_axis_value(self, pack_dict):
        for dim in (YAW_NAME, PITCH_NAME, ROLL_NAME, SWAY_NAME, SURGE_NAME, HEAVE_NAME):
            pack_dict[dim] = self.imu_to_six_axis_value(dim, pack_dict[dim])



def rospy_main():
    equalizer = Equalizer("./configs/equalizer_config.yaml")
    recorder_imu = Recorder("./configs/recorder_config.yaml")
    recorder_output = Recorder("./configs/recorder_config.yaml")
    imu_motion_controller = IMUMotionController(
        equalizer=equalizer, recorder_imu=recorder_imu, recorder_output=recorder_output
    )
    imu_motion_controller.do_equalization = True
    try:
        imu_motion_controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        imu_motion_controller.stop()


if __name__ == "__main__":
    rospy_main()
