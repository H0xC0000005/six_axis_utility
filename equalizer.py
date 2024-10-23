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

    def __init__(self, config_path):
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        k_yaw = config[YAW_NAME]["k"]
        self.prev_velocities_yaw = deque(maxlen=k_yaw + 1)
        self.prev_positions_yaw = deque(maxlen=k_yaw + 2)
        self.compressors = {}
        self.prev_dim_values = {}
        self.clamp_thresholds = {}
        max_k = -1
        for dim_name in self.dim_names:
            cur_k = config[dim_name]["k"]
            self.prev_dim_values[dim_name] = deque(maxlen=cur_k)
            max_k = max(max_k, cur_k)
            if COMPRESSOR_NAME not in config[dim_name]:
                self.compressors[dim_name] = None
            else:
                try:
                    threshold, ratio, attack, release, knee_width, bias = (
                        config[dim_name][COMPRESSOR_NAME][COMPRESSOR_THRESHOLD_NAME],
                        config[dim_name][COMPRESSOR_NAME][COMPRESSOR_RATIO_NAME],
                        config[dim_name][COMPRESSOR_NAME][COMPRESSOR_ATTACK_NAME],
                        config[dim_name][COMPRESSOR_NAME][COMPRESSOR_RELEASE_NAME],
                        config[dim_name][COMPRESSOR_NAME][COMPRESSOR_KNEEWIDTH_NAME],
                        config[dim_name][COMPRESSOR_NAME][COMPRESSOR_BIAS_NAME],
                    )
                    self.compressors[dim_name] = DynamicRangeCompressor(
                        threshold, ratio, attack, release, knee_width, bias
                    )
                except KeyError as e:
                    print(e)
                    raise ValueError(
                        f"if you want to use a dynamic range compressor, you must specify all the specifications."
                    )
            if CLAMP_THRESHOLD_NAME in config[dim_name]:
                self.clamp_thresholds[dim_name] = config[dim_name][CLAMP_THRESHOLD_NAME]
            else:
                self.clamp_thresholds[dim_name] = float("inf")
            pass
        self.prev_timesteps = deque(maxlen=max_k)
        self.config = config

    def is_dim_enabled(self, dim_name):
        cur_dim_config = self.config[dim_name]
        if ENABLE_DIM_NAME not in cur_dim_config or self.config[dim_name] == True:
            return True
        else:
            return False

    def update_time(self, new_time):
        self.sec_last_time = self.last_time
        self.last_time = self.cur_time
        if not DEBUG or self.cur_time is None:
            self.cur_time = new_time
        else:
            self.cur_time += 1 / 40

    def convert_last_yaw_to_accel(self):
        if not self.is_dim_enabled(YAW_NAME):
            return 0
        acceleration = 0
        if len(self.prev_positions_yaw) > 2:
            delta = self.prev_timesteps[-1] - self.prev_timesteps[-2]
            cur_yaw_velocity = (
                self.prev_positions_yaw[-1] - self.prev_positions_yaw[-2]
            ) / delta
            self.prev_velocities_yaw.append(cur_yaw_velocity)
            if len(self.prev_positions_yaw) > 3:
                window_length = min(len(self.prev_velocities_yaw), 10)
                if window_length % 2 == 0:
                    window_length -= 1
                smoothed_velocities = savgol_filter(
                    self.prev_velocities_yaw,
                    window_length=window_length,
                    polyorder=min(len(self.prev_velocities_yaw), 2) - 1,
                )
                acceleration = (smoothed_velocities[-1] - smoothed_velocities[-2]) / (
                    (self.prev_timesteps[-1] - self.prev_timesteps[-3]) / 2
                )
        return acceleration

    def clamp_value_dim(self, dim_name, *, value=None, inplace=True):
        if value is None:
            value = self.prev_dim_values[dim_name][-1]
        if len(self.prev_dim_values[dim_name]) < 2:
            return value
        if (
            value - self.prev_dim_values[dim_name][-2]
            >= self.clamp_thresholds[dim_name]
        ):
            value = self.prev_dim_values[dim_name][-2] + self.clamp_thresholds[dim_name]
        elif (
            value - self.prev_dim_values[dim_name][-2]
            <= -self.clamp_thresholds[dim_name]
        ):
            value = self.prev_dim_values[dim_name][-2] - self.clamp_thresholds[dim_name]
        if inplace:
            self.prev_dim_values[dim_name][-1] = value
        return value

    def clamp_last_values(self, *, values=None, inplace=True):
        result = {}
        for dim in self.dim_names:
            cur_value = None if values is None else values[dim]
            result[dim] = self.clamp_value_dim(dim, value=cur_value, inplace=inplace)
        return result

    def update_last_values(self, values_dict):
        for dim_name in self.dim_names:
            if dim_name == YAW_NAME:
                yaw_val = values_dict[YAW_NAME]
                self.prev_positions_yaw.append(yaw_val)
                yaw_accel = self.convert_last_yaw_to_accel()
                self.prev_dim_values[dim_name].append(yaw_accel)
            else:
                self.prev_dim_values[dim_name].append(values_dict[dim_name])
        self.prev_timesteps.append(values_dict[TIMESTAMP_NAME])

    def normalize_dim(self, dim_name, *, value=None, inplace=False):
        if not self.is_dim_enabled(dim_name):
            return 0
        if FILTERMODE_ITEM_NAME not in self.config[dim_name]:
            return self.prev_dim_values[dim_name][-1]
        cur_config = self.config[dim_name][FILTERMODE_ITEM_NAME]
        mode = cur_config[MODE_NAME]
        if mode == IDENTITY_MODE_NAME:
            return self.prev_dim_values[dim_name][-1]
        try:
            if value is None:
                value = self.prev_dim_values[dim_name]
            if mode == EMA_NAME:
                ret = apply_EMA_lastvalue(value, cur_config[ALPHA_NAME])
            elif mode == MA_NAME:
                return apply_MA_lastvalue(self.prev_dim_values[dim_name])
            elif mode == BUTTER_NAME:
                if ORDER_NAME in cur_config:
                    order = cur_config[ORDER_NAME]
                else:
                    order = 2
                ret = apply_Butterworth_lastvalue(
                    value, cur_config[CUTOFF_NAME], cur_config[FS_NAME], order
                )
            elif mode == WIENER_NAME:
                if NOISE_NAME in cur_config:
                    noise = cur_config[NOISE_NAME]
                else:
                    noise = None
                ret = apply_Wiener_lastvalue(
                    value, len(self.prev_dim_values[dim_name]), noise
                )
            elif mode == KALMAN_NAME:
                R = cur_config[KALMAN_R_NAME]
                Q = cur_config[KALMAN_Q_NAME]
                ret = apply_Kalman_lastvalue(value, R, Q)
            else:
                raise ValueError(
                    f"unknown mode provided: {mode}. supported modes are: {AVAILABLE_MODES}"
                )
        except Exception as e:
            print(f"<<< Error occured when normalizing dim {dim_name}.")
            raise e
        if inplace:
            self.prev_dim_values[dim_name] = ret
        return ret

    def normalize(self, *, values=None, inplace=False):
        result = {}
        for dim in self.dim_names:
            value = None if values is None else values[dim]
            cur_dim_result = self.normalize_dim(dim, value=value, inplace=inplace)
            result[dim] = cur_dim_result
        return result

    def apply_gain(self, values):
        for dim in self.dim_names:
            self.apply_gain_dim(values, dim)

    def apply_gain_dim(self, values, dim_name):
        if not self.is_dim_enabled(dim_name):
            return
        bias = 0
        if COMPRESSOR_NAME in self.config[dim_name]:
            bias = self.config[dim_name][COMPRESSOR_NAME][COMPRESSOR_BIAS_NAME]
        if GAIN_NAME in self.config[dim_name]:
            gain = self.config[dim_name][GAIN_NAME]
        else:
            gain = DEFAULT_GAIN
        values[dim_name] = bias + (values[dim_name] - bias) * gain

    def compress(self, *, values=None, inplace=False):
        result = {}
        for dim in self.dim_names:
            cur_value = (
                values[dim] if values is not None else self.prev_dim_values[dim][-1]
            )
            result[dim] = self.compress_dim(dim, value=cur_value, inplace=inplace)
        return result

    def compress_dim(self, dim_name, *, value=None, inplace=True):
        if value is None:
            value = self.prev_dim_values[dim_name][-1]
        if self.compressors[dim_name] is not None:
            ret = self.compressors[dim_name].compress(value)
        else:
            ret = value
        if inplace:
            self.prev_dim_values[dim_name][-1] = ret
        return ret

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

    def equalize_pipeline(self, latest_values_dict):
        self.update_last_values(latest_values_dict)
        self.compress(inplace=True)
        result = self.normalize(values=None, inplace=False)
        result = self.clamp_last_values(inplace=True)
        self.apply_gain(result)
        result[TIMESTAMP_NAME] = self.prev_timesteps[-1]
        return result


def prepare_test_data():
    file_path = "./exploration/imu_data_2.csv"
    df = pd.read_csv(file_path)
    return df


def test_equalizer_console():
    eq = Equalizer(
        "D:\\repos\\carla side code\\six_axis_utility\\equalizer_config.yaml"
    )

    def generate_test_data():
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
    time = np.linspace(0, 1, 1000)
    df = prepare_test_data()
    yaw, pitch, roll = quaternion_to_euler(
        [
            df["orientation_x"],
            df["orientation_y"],
            df["orientation_z"],
            df["orientation_w"],
        ]
    )
    original_signal = df["linear_acceleration_z"]
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
    dim_to_test = HEAVE_NAME
    plt.figure(figsize=(12, 6))
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
