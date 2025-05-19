from typing import Iterable, Sequence, Sized
import time
import math
import socket
import zmq


from control_sender import CustomGamePack, ControlSender, GameStatus, send_pack
from hyperparams import *
from utilities import *
from equalizer import Equalizer
from recorder import Recorder

# TODO: six axis platform machine specs hardcoded. may need refactoring
IP_ADDRESS = "192.168.1.239"
PORT = 15620

# ZMQ ports
IMU_PORT = 5560
COL_PORT = 5562

class IMUMotionController:
    def __init__(
        self,
        equalizer: Equalizer | None = None,
        recorder_imu: Recorder | None = None,
        recorder_output: Recorder | None = None,
    ):
        self.context = zmq.Context()
        # Subscribe to IMU and collision
        self.imu_sub = self.context.socket(zmq.SUB)
        self.imu_sub.connect(f"tcp://127.0.0.1:{IMU_PORT}")
        self.imu_sub.setsockopt(zmq.SUBSCRIBE, b"")               # All IMU data :contentReference[oaicite:22]{index=22}

        self.col_sub = self.context.socket(zmq.SUB)
        self.col_sub.connect(f"tcp://127.0.0.1:{COL_PORT}")
        self.col_sub.setsockopt(zmq.SUBSCRIBE, b"")               # All collision data :contentReference[oaicite:23]{index=23}

        self.poller = zmq.Poller()
        self.poller.register(self.imu_sub, zmq.POLLIN)
        self.poller.register(self.col_sub, zmq.POLLIN)

        self.control_sender = ControlSender()
        
        self.end_point = (IP_ADDRESS, PORT)
        self.udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.equalizer = equalizer
        self.do_equalization = True

        self.recorder_imu = recorder_imu
        self.recorder_output = recorder_output

        # raw imu message handlers. BEWARE OF THE ORDER OF CHAIN OF RESPONSIBILITY
        self.raw_imu_json_handler = [process_carla_imu_message, process_identity_imu_message]

        # ------------------------------------
        # DEBUG
        self.recv_cnt = 0
        self.tot_time = 1e-15
        # ------------------------------------


        print(f"IMU motion controller init finished.")

    def spin(self):
        while True:
            start_time = time.time()
            events = dict(self.poller.poll(timeout=5))            # Poll both sockets :contentReference[oaicite:24]{index=24}
            if self.imu_sub in events:
                self.recv_cnt += 1
                msg = self.imu_sub.recv_json()
                print(f"received imu msg / telemetry: {msg}")
                # Process IMU data
                # TODO: add universal support of input processing
                # result_dict = process_carla_imu_message(msg)
                for handler in self.raw_imu_json_handler:
                    try:
                        result_dict = handler(msg)
                        break
                    except (AttributeError, KeyError):
                        continue
                else:
                    raise ValueError(f"received imu message cannot be parsed by any of the handlers: {msg}")
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
                send_pack(self.udp_client, self.end_point, result)    # Send to six-axis platform :contentReference[oaicite:25]{index=25}
                print(f"sent game pack: {result_dict}")
            if self.col_sub in events:
                collide = self.col_sub.recv_json()
                print(f"Collision detected: {collide}")
                self.equalizer.register_collision_frame()            # Handle collision callback
            end_time = time.time()
            self.tot_time += end_time - start_time
            print(f"current recv frame rate: {self.recv_cnt / self.tot_time}")


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
        imu_motion_controller.spin()
    except KeyboardInterrupt:
        print(f"<<< keyboard interrupt received from user.")
    finally:
        imu_motion_controller.stop()



if __name__ == "__main__":
    main()
