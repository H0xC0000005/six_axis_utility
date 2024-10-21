"""
functions of equalizer:
1: convert yaw from position to acceleration
2: platform filters pulse value. need to use a low pass filter to smooth
3: channelwise transform, currently including scaling and low pass filtering
"""

from collections import deque
import time
from matplotlib import pyplot as plt
import pandas as pd
import yaml
import random

from scipy.signal import savgol_filter

from constants import *
from hyperparams import *
from utilities import *
from filters import *


class Equalizer:
    dim_names = (YAW_NAME, PITCH_NAME, ROLL_NAME, SWAY_NAME, SURGE_NAME, HEAVE_NAME)
    clamp_threshold = 1.5

    def __init__(self, config_path: str):
        """
        Args:
            config_path: the .yaml config file path.
        """
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        k_yaw = config["Yaw"]["k"]

        # self.prev_velocities_yaw = deque(maxlen=k_yaw - 1)
        # self.prev_accelerations_yaw = deque(maxlen=k_yaw - 2)
        self.prev_velocities_yaw = deque(maxlen=k_yaw + 1)
        self.prev_positions_yaw = deque(maxlen=k_yaw + 2)

        self.prev_dim_values = {}
        max_k = -1
        for dim_name in self.dim_names:
            cur_k = config[dim_name]["k"]
            self.prev_dim_values[dim_name] = deque(maxlen=cur_k)
            max_k = max(max_k, cur_k)
        self.prev_timesteps = deque(maxlen=max_k)
        self.config = config

    def is_dim_enabled(self, dim_name: str):
        """
        check whether a dimension is in the output.
        if the config is missing, by default it is in the output.
        """
        cur_dim_config = self.config[dim_name]
        if ENABLE_DIM_NAME not in cur_dim_config or self.config[dim_name] == True:
            return True
        else:
            return False

    def update_time(self, new_time: float):
        """
        since there are multiple calculations, instance storage is used
        to sync the time for each signal arrival
        DEPRECATED AFTER ADDING TIME TO CUR STEP DICT.
        RETAINED FOR DEBUG.
        """
        self.sec_last_time = self.last_time
        self.last_time = self.cur_time
        if not DEBUG or self.cur_time is None:
            self.cur_time = new_time
        else:
            # assume that it is 40 frames per second
            self.cur_time += 1 / 40

    def convert_last_yaw_to_accel(self) -> float:
        """
        convert yaw position reading to acceleration,
        and update yaw acceleration computation caches
        """
        if not self.is_dim_enabled(YAW_NAME):
            return 0
        acceleration = 0
        if (
            len(self.prev_positions_yaw) > 2
        ):  # only compute accel if we have 3+ positions, and vel if 2+
            delta = self.prev_timesteps[-1] - self.prev_timesteps[-2]
            # assume that yaw is in radian. retrieve
            cur_yaw_velocity = (
                self.prev_positions_yaw[-1] - self.prev_positions_yaw[-2]
            ) / delta
            self.prev_velocities_yaw.append((cur_yaw_velocity))
            if len(self.prev_positions_yaw) > 3:
                # apply a savgol filter to velocity before computing acceleration
                smoothed_velocities = savgol_filter(
                    self.prev_velocities_yaw,
                    window_length=min(len(self.prev_velocities_yaw), 10),
                    polyorder=min(len(self.prev_velocities_yaw), 2) - 1,
                )
                # TODO: extend the acceleration window, not just using the last two velocities
                # TODO: add timestamp support in data passing
                # UPDATE: compute acceleration directly from positions
                acceleration = (smoothed_velocities[-1] - smoothed_velocities[-2]) / (
                    (self.prev_timesteps[-1] - self.prev_timesteps[-3]) / 2
                )
                # acceleration = (
                #     self.prev_velocities_yaw[-1] - self.prev_velocities_yaw[-2]
                # ) / ((self.prev_timesteps[-1] - self.prev_timesteps[-3]) / 2)
                """
                clamp the acceleration; 
                the platform has physical constraint, and a large acceleration is not reasonable itself
                """
                if (
                    acceleration - self.prev_dim_values[YAW_NAME][-1]
                    >= self.clamp_threshold
                ):
                    acceleration = (
                        self.prev_dim_values[YAW_NAME][-1] + self.clamp_threshold
                    )
                elif (
                    acceleration - self.prev_dim_values[YAW_NAME][-1]
                    <= -self.clamp_threshold
                ):
                    acceleration = (
                        self.prev_dim_values[YAW_NAME][-1] - self.clamp_threshold
                    )
        return acceleration

    def update_last_values(self, values_dict: dict[str, float]):
        """
        update the recorded values.
        notice that for yaw, conversion of yaw positions to accelerations is
        done here.
        """
        for dim_name in self.dim_names:
            if dim_name == YAW_NAME:
                # need to convert position to acceleration for yaw
                yaw_val = values_dict[YAW_NAME]
                self.prev_positions_yaw.append(yaw_val)
                yaw_accel = self.convert_last_yaw_to_accel()
                self.prev_dim_values[dim_name].append(yaw_accel)
            else:
                self.prev_dim_values[dim_name].append(values_dict[dim_name])
        # update timestep. added in v1.2
        self.prev_timesteps.append(values_dict[TIMESTAMP_NAME])

    def normalize_dim(self, dim_name: str) -> float:
        """
        produce the normalized value for the latest timestep (for control)
        """
        if not self.is_dim_enabled(dim_name):
            return 0
        if FILTERMODE_ITEM_NAME not in self.config[dim_name]:
            # filter mode not specified, by default apply identity filter
            return self.prev_dim_values[dim_name][-1]
        cur_config = self.config[dim_name][FILTERMODE_ITEM_NAME]
        mode = cur_config[MODE_NAME]
        if mode == IDENTITY_MODE_NAME:
            return self.prev_dim_values[dim_name][-1]
        try:
            if mode == EMA_NAME:
                return apply_EMA_lastvalue(
                    self.prev_dim_values[dim_name],
                    cur_config[ALPHA_NAME],
                )
            elif mode == MA_NAME:
                return apply_MA_lastvalue(self.prev_dim_values[dim_name])
            elif mode == BUTTER_NAME:
                if ORDER_NAME in cur_config:
                    order = cur_config[ORDER_NAME]
                else:
                    order = 2
                return apply_Butterworth_lastvalue(
                    self.prev_dim_values[dim_name],
                    cur_config[CUTOFF_NAME],
                    cur_config[FS_NAME],
                    order,
                )
            elif mode == WIENER_NAME:
                if NOISE_NAME in cur_config:
                    noise = cur_config[NOISE_NAME]
                else:
                    noise = None
                return apply_Wiener_lastvalue(
                    self.prev_dim_values[dim_name],
                    len(self.prev_dim_values[dim_name]),
                    noise,
                )
            elif mode == KALMAN_NAME:
                R = cur_config[KALMAN_R_NAME]
                Q = cur_config[KALMAN_Q_NAME]
                return apply_Kalman_lastvalue(self.prev_dim_values[dim_name], R, Q)
            else:
                raise ValueError(
                    f"unknown mode provided: {mode}. supported modes are: {AVAILABLE_MODES}"
                )
        except Exception as e:
            print(f"<<< Error occured when normalizing dim {dim_name}.")
            raise e

    def normalize(self) -> dict[str, float]:
        result = {}
        for dim in self.dim_names:
            result[dim] = self.normalize_dim(dim)
        return result

    def apply_gain(self, values: dict[str, float]):
        """
        apply gain for each dimension.
        in-place since I don't want to mess the original data as this operation is iterative
        and changes the scale.
        """
        for dim in self.dim_names:
            self.apply_gain_dim(values, dim)

    def apply_gain_dim(self, values: dict[str, float], dim_name: str):
        if not self.is_dim_enabled(dim_name):
            return
        if GAIN_NAME in self.config[dim_name]:
            gain = self.config[dim_name][GAIN_NAME]
        else:
            gain = DEFAULT_GAIN
        values[dim_name] *= gain

    def equalize_pipeline_from_carla_imu(self, msg):
        try:
            latest_values_dict = convert_carla_imu_message_to_dict(msg)
        except Exception as e:
            print(e)
            print(
                f"WARNING: cannot convert carla interface to dict. treating msg as dict..."
            )
            latest_values_dict = msg
        return self.equalize_pipeline(latest_values_dict)

    def equalize_pipeline(self, latest_values_dict: dict[str, float]):
        self.update_last_values(latest_values_dict)
        # since this class use internal storage, i use push & update apporach
        result = self.normalize()
        self.apply_gain(result)
        return result


"""
test functions
"""


def prepare_test_data():

    # Load the CSV file
    file_path = "D:\\repos\\carla side code\\six_axis_utility\\exploration\\test.csv"
    df = pd.read_csv(file_path)
    return df


def test_equalizer_console():

    eq = Equalizer(
        "D:\\repos\\carla side code\\six_axis_utility\\equalizer_config.yaml"
    )

    def generate_test_data() -> dict[str, float]:
        cur_val = random.random()
        return {x: cur_val for x in Equalizer.dim_names}

    cnt = 0
    while True:
        print(f"------------------ current count: {cnt}")
        cur_data = generate_test_data()
        print(f"cur data: {cur_data}")
        eq_data = eq.equalize_pipeline(cur_data)
        print(f"equalized data: {eq_data}")
        cnt += 1
        time.sleep(1)


def test_equalizer_plot():
    equalizer = Equalizer(
        "D:\\repos\\carla side code\\six_axis_utility\\equalizer_config.yaml"
    )
    # Simulated real-time signal input
    time = np.linspace(0, 1, 1000)
    # original_signal = np.sin(2 * np.pi * 5 * time) * 0.8 + np.random.randn(1000) * 0.05
    # original_signal[100:150] += 0.5  # Adding a sudden peak for testing
    # original_signal[200:250] -= 0.5  # Adding a sudden negative peak for testing
    # original_signal = np.sin(2 * np.pi * 5 * time) * 0.8 + np.random.randn(1000) * 0.01
    df = prepare_test_data()
    yaw, pitch, roll = quaternion_to_euler(
        [
            df["orientation_x"],
            df["orientation_y"],
            df["orientation_z"],
            df["orientation_w"],
        ]
    )
    original_signal = yaw

    # Process each value in the signal one at a time
    compressed_signal = {
        dim_name: np.zeros_like(original_signal) for dim_name in Equalizer.dim_names
    }
    for i in range(len(original_signal)):
        cur_data = {dim_name: original_signal[i] for dim_name in Equalizer.dim_names}
        cur_data[TIMESTAMP_NAME] = df["timestamp"][i]
        cur_result = equalizer.equalize_pipeline(cur_data)
        for dim_name in Equalizer.dim_names:
            compressed_signal[dim_name][i] = cur_result[dim_name]
            pass

    dim_to_test = YAW_NAME
    # Plot the result
    plt.figure(figsize=(12, 6))
    # plt.plot(time, original_signal, label="Original Signal")
    plt.plot(df["timestamp"], original_signal, label="Original Signal")
    plt.plot(
        df["timestamp"],
        compressed_signal[dim_to_test],
        label=f"Compressed Signal: {dim_to_test}",
        linewidth=1,
    )
    plt.xlabel("Time")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.title("equalizer testing plot")
    plt.show()


if __name__ == "__main__":
    test_equalizer_plot()
