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

    def _init_yaw_gamma_filter(self):
        # Initialize Alpha-Beta-Gamma filter parameters if not already done
        if not hasattr(self, "alpha_beta_gamma_state"):
            # State: [position, velocity, acceleration]
            self.alpha_beta_gamma_state = np.array([0.0, 0.0, 0.0])
            self.alpha = 0.85  # Alpha: Position update weight
            self.beta = 0.005  # Beta: Velocity update weight
            self.gamma = 0.0001  # Gamma: Acceleration update weight
            self.prev_time = None

    def _init_yaw_kalman_filter(self):
        # Initialize Kalman Filter if not already done
        # Initialize Kalman Filter if not already done
        if not hasattr(self, "kalman_filter"):
            # 1D Constant Jerk Model (position, velocity, acceleration, jerk)
            self.kalman_filter = KalmanFilter(dim_x=4, dim_z=1)
            dt = 1.0  # Initial time step estimate (this will vary later)

            # State transition matrix (position, velocity, acceleration, jerk)
            self.kalman_filter.F = np.array(
                [
                    [1, dt, 0.5 * dt**2, (1 / 6) * dt**3],
                    [0, 1, dt, 0.5 * dt**2],
                    [0, 0, 1, dt],
                    [0, 0, 0, 1],
                ]
            )

            # Measurement function (we only measure position)
            self.kalman_filter.H = np.array([[1, 0, 0, 0]])

            # Initial covariance matrix
            self.kalman_filter.P *= 1000.0

            # Process uncertainty - Increased to be more responsive for acceleration and jerk
            self.kalman_filter.Q = (
                np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 5, 0], [0, 0, 0, 10]])
                * 0.1
            )

            # Measurement uncertainty
            self.kalman_filter.R = np.array(
                [[1]]
            )  # Lowered measurement noise to trust the sensor more

            # Initial state (position, velocity, acceleration, jerk)
            self.kalman_filter.x = np.array([[0], [0], [0], [0]])
        else:
            print(
                f"WARNING: attempt to initialize yaw kalman filter but it already presents. return without any operation"
            )

    def __init__(self, config_path: str):
        """
        Args:
            config_path: the .yaml config file path.
        """
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        k_yaw = config[YAW_NAME]["k"]

        # self.prev_velocities_yaw = deque(maxlen=k_yaw - 1)
        # self.prev_accelerations_yaw = deque(maxlen=k_yaw - 2)
        # self.prev_velocities_yaw = deque(maxlen=k_yaw + 1)
        self.prev_positions_yaw = deque(maxlen=k_yaw + 2)
        self._init_yaw_gamma_filter()
        self._init_yaw_kalman_filter()
        self.compressors = {}
        self.prev_dim_values = {}
        self.clamp_thresholds = {}
        max_k = k_yaw + 2
        for dim_name in self.dim_names:
            # process k and deque
            cur_k = config[dim_name]["k"]
            self.prev_dim_values[dim_name] = deque(maxlen=cur_k)
            max_k = max(max_k, cur_k)
            # process compressors
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
            # process clamp threshold

            if CLAMP_THRESHOLD_NAME in config[dim_name]:
                self.clamp_thresholds[dim_name] = config[dim_name][CLAMP_THRESHOLD_NAME]
            else:
                self.clamp_thresholds[dim_name] = float("inf")
            pass

        self.prev_timesteps = deque(maxlen=max_k)
        self.config = config

    def is_dim_enabled(self, dim_name: str):
        """
        check whether a dimension is in the output.
        if the config is missing, by default it is in the output.
        """
        cur_dim_config = self.config[dim_name]
        if (
            ENABLE_DIM_NAME not in cur_dim_config
            or self.config[dim_name][ENABLE_DIM_NAME] == True
        ):
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
        # Update the time step based on the most recent timestamps
        if len(self.prev_timesteps) >= 2:
            dt = self.prev_timesteps[-1] - self.prev_timesteps[-2]

            # Update state transition matrix with the new time step
            self.kalman_filter.F = np.array(
                [
                    [1, dt, 0.5 * dt**2, (1 / 6) * dt**3],
                    [0, 1, dt, 0.5 * dt**2],
                    [0, 0, 1, dt],
                    [0, 0, 0, 1],
                ]
            )

            # Adaptive tuning for process noise based on time step
            self.kalman_filter.Q = (
                np.array(
                    [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 5 * dt, 0], [0, 0, 0, 10 * dt]]
                )
                * 0.1
            )

        # Get the most recent position
        latest_position = self.prev_positions_yaw[-1]

        # Kalman filter prediction and update
        self.kalman_filter.predict()
        self.kalman_filter.update(latest_position)

        # The estimated acceleration is the third element in the state vector
        acceleration = self.kalman_filter.x[2, 0]
        return acceleration

    def clamp_value_dim(
        self,
        dim_name: str,
        *,
        value: float | None = None,
        inplace: bool = True,
    ):
        """
        clamp the specified dim.
        this is done on updated data, and can be inplace, i.e. store the clamped value as the latest element.
        """
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

    def clamp_last_values(
        self,
        *,
        values: dict[str, float] | None = None,
        inplace: bool = True,
    ):
        result = {}
        for dim in self.dim_names:
            cur_value = None if values is None else values[dim]
            result[dim] = self.clamp_value_dim(dim, value=cur_value, inplace=inplace)
        return result

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

    def normalize_dim(
        self, dim_name: str, *, value=None, inplace: bool = False
    ) -> float:
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
            if value is None:
                value = self.prev_dim_values[dim_name]
            if mode == EMA_NAME:
                ret = apply_EMA_lastvalue(
                    value,
                    cur_config[ALPHA_NAME],
                )
            elif mode == MA_NAME:
                return apply_MA_lastvalue(self.prev_dim_values[dim_name])
            elif mode == BUTTER_NAME:
                if ORDER_NAME in cur_config:
                    order = cur_config[ORDER_NAME]
                else:
                    order = 2
                ret = apply_Butterworth_lastvalue(
                    value,
                    cur_config[CUTOFF_NAME],
                    cur_config[FS_NAME],
                    order,
                )
            elif mode == WIENER_NAME:
                if NOISE_NAME in cur_config:
                    noise = cur_config[NOISE_NAME]
                else:
                    noise = None
                ret = apply_Wiener_lastvalue(
                    value,
                    len(self.prev_dim_values[dim_name]),
                    noise,
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

    def normalize(
        self, *, values: dict[str, float] | None = None, inplace: bool = False
    ) -> dict[str, float]:
        result = {}
        for dim in self.dim_names:
            value = None if values is None else values[dim]
            cur_dim_result = self.normalize_dim(dim, value=value, inplace=inplace)
            result[dim] = cur_dim_result
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
        """
        apply gain for the specified dimension.
        the gain is calculated around the bias. the bias is by default 0.
        """
        if not self.is_dim_enabled(dim_name):
            return
        bias = 0
        # TODO: wierd logic wiring. need to consider whether can we refactor the bias
        # outside the compressor, since the bias may be used by multiple components in the equalizer
        if COMPRESSOR_NAME in self.config[dim_name]:
            # HACK: if compressor is specified, all specs of the parameter are there by requirement.
            bias = self.config[dim_name][COMPRESSOR_NAME][COMPRESSOR_BIAS_NAME]
        if GAIN_NAME in self.config[dim_name]:
            gain = self.config[dim_name][GAIN_NAME]
        else:
            gain = DEFAULT_GAIN
        # values[dim_name] *= gain
        values[dim_name] = bias + (values[dim_name] - bias) * gain

    def compress(
        self, *, values: dict[str, float] | None = None, inplace: bool = False
    ) -> dict[str, float]:
        result = {}
        for dim in self.dim_names:
            cur_value = (
                values[dim] if values is not None else self.prev_dim_values[dim][-1]
            )
            result[dim] = self.compress_dim(dim, value=cur_value, inplace=inplace)
        return result

    def compress_dim(
        self, dim_name: str, *, value: float | None = None, inplace: bool = True
    ) -> float:
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

    def equalize_pipeline(self, latest_values_dict: dict[str, float]):
        self.update_last_values(latest_values_dict)
        # first compress, then normalize
        # result = self.compress(inplace=False)
        self.compress(inplace=True)
        # since this class use internal storage, i use push & update apporach
        # result = self.normalize(values=result, inplace=False)
        result = self.normalize(values=None, inplace=False)
        # apply clamp after all processing and before gain
        result = self.clamp_last_values(values=result, inplace=False)
        self.apply_gain(result)
        return result


"""
test functions
"""


def prepare_test_data():
    file_path = "./exploration/imu_data_1.csv"
    df = pd.read_csv(file_path)
    return df


def prepare_recorded_data():
    file_path = "exploration/imu_1729673492.0006297.csv"
    df = pd.read_csv(file_path)
    return df


def test_equalizer_console():

    eq = Equalizer("./configs/equalizer_config.yaml")

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
    equalizer = Equalizer("./configs/equalizer_config.yaml")
    # Simulated real-time signal input
    time = np.linspace(0, 1, 1000)
    # original_signal = np.sin(2 * np.pi * 5 * time) * 0.8 + np.random.randn(1000) * 0.05
    # original_signal[100:150] += 0.5  # Adding a sudden peak for testing
    # original_signal[200:250] -= 0.5  # Adding a sudden negative peak for testing
    # original_signal = np.sin(2 * np.pi * 5 * time) * 0.8 + np.random.randn(1000) * 0.01
    # df = prepare_test_data()
    # yaw, pitch, roll = quaternion_to_euler(
    #     [
    #         df["orientation_x"],
    #         df["orientation_y"],
    #         df["orientation_z"],
    #         df["orientation_w"],
    #     ]
    # )
    df = prepare_recorded_data()

    dim_to_test = HEAVE_NAME
    # original_signal = yaw
    # original_signal = df["linear_acceleration_z"]
    original_signal = df[dim_to_test]

    # Process each value in the signal one at a time
    compressed_signal = {
        dim_name: np.zeros_like(original_signal) for dim_name in Equalizer.dim_names
    }
    for i in range(len(original_signal)):
        cur_data = {dim_name: original_signal[i] for dim_name in Equalizer.dim_names}
        cur_data[TIMESTAMP_NAME] = df[TIMESTAMP_NAME][i]
        cur_result = equalizer.equalize_pipeline(cur_data)
        for dim_name in Equalizer.dim_names:
            compressed_signal[dim_name][i] = cur_result[dim_name]
            pass

    # Plot the result
    plt.figure(figsize=(12, 6))
    # plt.plot(time, original_signal, label="Original Signal")
    plt.plot(df[TIMESTAMP_NAME], original_signal, label="Original Signal")
    plt.plot(
        df[TIMESTAMP_NAME],
        compressed_signal[dim_to_test],
        label=f"Processed Signal: {dim_to_test}",
        linewidth=1,
    )
    plt.xlabel("Time")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.title("equalizer testing plot")
    plt.show()


if __name__ == "__main__":
    test_equalizer_plot()
