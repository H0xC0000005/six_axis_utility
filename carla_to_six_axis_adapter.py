from typing import Iterable, Sequence, Sized
import rclpy
from rclpy.node import Node

try:
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Vector3
except ImportError:
    print("!!!WARNING: sensor_msgs or geometry_msgs not available.")
import time
import math
import socket


from control_sender import CustomGamePack, ControlSender, GameStatus, send_pack
from hyperparams import *
from utilities import *
from equalizer import Equalizer
from recorder import Recorder

# TODO: six axis platform machine specs hardcoded. may need refactoring
IP_ADDRESS = "192.168.0.208"
PORT = 15620

class IMUMotionController(Node):
    def __init__(
        self,
        equalizer: Equalizer | None = None,
        recorder_imu: Recorder | None = None,
        recorder_output: Recorder | None = None,
    ):
        super().__init__("imu_motion_controller")

        # Create subscriptions using ROS2's create_subscription method
        self.subscription = self.create_subscription(
            Imu, "/imu/imu", self.imu_callback, 10
        )
        self.collision_subscription = self.create_subscription(
            Vector3, "/collision_detector", self.collision_callback, 10
        )

        self.control_sender = ControlSender()
        
        self.end_point = (IP_ADDRESS, PORT)
        self.udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.equalizer = equalizer
        self.do_equalization = True

        self.recorder_imu = recorder_imu
        self.recorder_output = recorder_output
        print(f"IMU motion controller init finished.")

    def imu_callback(self, msg):
        print(f">>")
        # Process IMU data
        result_dict = convert_carla_imu_message_to_dict(msg)
        if self.recorder_imu is not None:
            self.recorder_imu.update_record(result_dict)
        self.pack_imu_to_six_axis_value(result_dict)
        if self.do_equalization and self.equalizer is not None:
            result_dict = self.equalizer.equalize_pipeline(result_dict)
        # self.multiply_imu(result_dict, (1, 0.1))
        result = CustomGamePack.from_dict(result_dict)
        result.GameStatus = GameStatus.GameStart
        if self.recorder_output is not None:
            self.recorder_output.update_record(result_dict)
        # Send telemetry to six axis platform
        send_pack(self.udp_client, self.end_point, result)

    def collision_callback(self, msg):
        print("Collision detected:")
        print("Force - x: %f, y: %f, z: %f", msg.x, msg.y, msg.z)
        self.equalizer.register_collision_frame()

    def stop(self):
        if self.recorder_imu is not None:
            self.recorder_imu.to_csv(f"./carla_data/imu_{time.time()}.csv")
        if self.recorder_output is not None:
            self.recorder_output.to_csv(f"./carla_data/output_{time.time()}.csv")

    @staticmethod
    def imu_to_six_axis_value(dim_name: str, reading: float) -> float:
        """
        convert imu reading from Unreal to six axis orientation parameter range. assume that imu reading is in radian
        and six axis orientation is in range [-k, k].
        """
        sign = 1
        # some of the axes have opposite control direction
        if dim_name in (PITCH_NAME, ROLL_NAME):
            sign = -1
        return reading / IMU_RANGES[dim_name] * SIX_AXIS_RANGES[dim_name] * sign

    def pack_imu_to_six_axis_value(self, pack_dict: dict[str, float]) -> None:
        for dim in (YAW_NAME, PITCH_NAME, ROLL_NAME, SWAY_NAME, SURGE_NAME, HEAVE_NAME):
            pack_dict[dim] = self.imu_to_six_axis_value(dim, pack_dict[dim])


def main(args=None):
    # Initialize ROS2 communication
    rclpy.init(args=args)
    equalizer = Equalizer("./configs/equalizer_config.yaml")
    # recorder_imu = Recorder("./configs/recorder_config.yaml")
    # recorder_output = Recorder("./configs/recorder_config.yaml")
    recorder_imu = None
    recorder_output = None
    imu_motion_controller = IMUMotionController(
        equalizer=equalizer, recorder_imu=recorder_imu, recorder_output=recorder_output
    )
    imu_motion_controller.do_equalization = True
    print(f"ready to launch the pipeline...")
    try:
        rclpy.spin(imu_motion_controller)
    except KeyboardInterrupt:
        print(f"<<< keyboard interrupt received from user.")
    finally:
        imu_motion_controller.stop()
        imu_motion_controller.destroy_node()


if __name__ == "__main__":
    main()
