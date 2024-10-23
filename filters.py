from collections import deque
from matplotlib import pyplot as plt
import numpy as np
from scipy.signal import butter, lfilter, wiener
from filterpy.kalman import KalmanFilter
from constants import *


def apply_EMA_lastvalue(arr, alpha):
    weights = np.array([((1 - alpha) ** (len(arr) - 1 - i)) for i in range(len(arr))])
    weights /= weights.sum()
    weighted_average = np.dot(weights, np.array(arr))
    if DEBUG_PRINT:
        print(f">>>> in apply EMA:")
        print(f">> weights: {weights}")
        print(f">> arr: {arr}")
        print(f">> weighted_avg: {weighted_average}")
        print(f">>>>")
    return weighted_average


def apply_MA_lastvalue(arr):
    average = sum(arr) / len(arr)
    return average


def apply_Butterworth_lastvalue(arr, cutoff, fs, order=2):
    b, a = butter(order, cutoff / (0.5 * fs), btype="low", analog=False)
    filtered_arr = lfilter(b, a, arr)
    return filtered_arr[-1]


class ButterworthFilter:

    def __init__(self, order, cutoff, sample_rate):
        self.order = order
        self.cutoff = cutoff
        self.sample_rate = sample_rate
        self.b, self.a = butter(order, cutoff / (0.5 * sample_rate), btype="low")
        self.zi = np.zeros(max(len(self.a), len(self.b)) - 1)

    def __call__(self, new_value):
        filtered_value, self.zi = lfilter(self.b, self.a, [new_value], zi=self.zi)
        return filtered_value[0]


def apply_Wiener_lastvalue(arr, k_wiener, noise=None):
    if len(arr) > 1:
        filtered_arr = wiener(arr, mysize=k_wiener, noise=noise)
        return filtered_arr[-1]
    else:
        return 0


class WienerFilter:

    def __init__(self, noise_estimate=0.1):
        self.signal_estimate = 0.0
        self.noise_estimate = noise_estimate
        self.alpha = 1.0

    def __call__(self, new_value):
        self.signal_estimate = (
            self.alpha * new_value**2 + (1 - self.alpha) * self.signal_estimate
        )
        snr_estimate = self.signal_estimate / (self.noise_estimate + 1e-09)
        wiener_gain = snr_estimate / (1 + snr_estimate)
        filtered_value = wiener_gain * new_value
        return filtered_value


def apply_Kalman_lastvalue(arr, R=0.01, Q=1e-05):
    kf = KalmanFilter(dim_x=1, dim_z=1)
    kf.x = np.array([[0.0]])
    kf.F = np.array([[1.0]])
    kf.H = np.array([[1.0]])
    kf.P = np.array([[1.0]])
    kf.R = R
    kf.Q = Q
    for value in arr:
        kf.predict()
        kf.update(value)
    return kf.x[0, 0]


class OneDimensionalKalmanFilter:

    def __init__(
        self,
        process_variance,
        measurement_variance,
        initial_estimate=0.0,
        initial_estimate_error=1.0,
    ):
        self.kf = KalmanFilter(dim_x=1, dim_z=1)
        self.kf.x = np.array([[initial_estimate]])
        self.kf.P = np.array([[initial_estimate_error]])
        self.kf.F = np.array([[1]])
        self.kf.H = np.array([[1]])
        self.kf.R = np.array([[measurement_variance]])
        self.kf.Q = np.array([[process_variance]])

    def update(self, measurement):
        self.kf.predict()
        self.kf.update(np.array([[measurement]]))
        return self.kf.x[0, 0]


class DynamicRangeCompressor:

    def __init__(
        self, threshold=0.5, ratio=3, attack=0.01, release=0.1, knee_width=0.1, bias=0
    ):
        assert threshold > 0, f"threshold must be positive"
        self.threshold = threshold
        assert ratio >= 1, f"ratio must be greater than 1"
        self.ratio = ratio
        assert attack > 0, f"attack must be positive"
        self.attack = attack
        assert release > 0, f"release must be positive"
        self.release = release
        assert knee_width > 0, f"knee width must be positive"
        self.knee_width = knee_width
        self.gain_reduction = 0
        self.bias = bias
        self.reset()

    def compress(self, signal_value):
        if not isinstance(signal_value, float) and not isinstance(signal_value, int):
            signal_value = signal_value[-1]
        abs_value = abs(signal_value - self.bias)
        if abs_value > self.threshold + self.knee_width / 2:
            excess = abs_value - self.threshold
            target_gain_reduction = (
                np.sign(signal_value) * excess * (1 - 1 / self.ratio)
            )
        elif abs_value > self.threshold - self.knee_width / 2:
            distance_into_knee = (
                abs_value - (self.threshold - self.knee_width / 2)
            ) / self.knee_width
            effective_ratio = 1 + (self.ratio - 1) * distance_into_knee**2
            excess = abs_value - self.threshold + self.knee_width / 2
            target_gain_reduction = (
                np.sign(signal_value) * excess * (1 - 1 / effective_ratio)
            )
        else:
            target_gain_reduction = 0
        if self.gain_reduction < target_gain_reduction:
            self.gain_reduction += self.attack
            if self.gain_reduction > target_gain_reduction:
                self.gain_reduction = target_gain_reduction
        elif self.gain_reduction > target_gain_reduction:
            self.gain_reduction -= self.release
            if self.gain_reduction < target_gain_reduction:
                self.gain_reduction = target_gain_reduction
        compressed_value = signal_value - self.gain_reduction
        return compressed_value

    def reset(self):
        self.gain_reduction = 0


def test_dynamic_range_compressor():
    sample_rate = 1000
    duration = 2
    frequency = 5
    time = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    sine_wave = np.sin(2 * np.pi * frequency * time)
    noise = np.random.normal(0, 0.1, sine_wave.shape)
    spikes = np.zeros_like(sine_wave)
    spike_indices = np.random.choice(len(sine_wave), size=10, replace=False)
    spikes[spike_indices] = np.random.uniform(-1, 1, size=10)
    noisy_sine_wave = sine_wave + noise + spikes
    compressor = DynamicRangeCompressor(
        threshold=0.5, ratio=4, attack=0.01, release=0.05, knee_width=0.2
    )
    compressed_wave = []
    for sample in noisy_sine_wave:
        compressed_sample = compressor.compress(sample)
        compressed_wave.append(compressed_sample)
    compressed_wave = np.array(compressed_wave)
    plt.figure(figsize=(12, 6))
    plt.plot(time, noisy_sine_wave, label="Original Signal", linewidth=1, linestyle="-")
    plt.plot(
        time, compressed_wave, label="Compressed Signal", linewidth=1, linestyle="-"
    )
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.title("Original and Compressed Sine Wave with Noise and Spikes")
    plt.legend()
    plt.grid()
    plt.show()


def test_EMA_filter():
    sample_rate = 1000
    duration = 2
    frequency = 5
    time = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    sine_wave = np.sin(2 * np.pi * frequency * time)
    noise = np.random.normal(0, 0.1, sine_wave.shape)
    spikes = np.zeros_like(sine_wave)
    spike_indices = np.random.choice(len(sine_wave), size=10, replace=False)
    spikes[spike_indices] = np.random.uniform(-1, 1, size=10)
    noisy_sine_wave = sine_wave + noise + spikes
    k = 6
    deq = deque(maxlen=k)
    result = []
    for elem in noisy_sine_wave:
        deq.append(elem)
        result.append(apply_EMA_lastvalue(deq, 0.3))
    result = np.array(result)
    plt.figure(figsize=(12, 6))
    plt.plot(time, noisy_sine_wave, label="original Signal", linewidth=1, linestyle="-")
    plt.plot(time, result, label="filtered Signal", linewidth=1, linestyle="-")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    test_EMA_filter()
