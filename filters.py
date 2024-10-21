"""
Signal filter candidates, used by the equalizer.
All signal processing utilities witch only refines data without changing its shape should be stored here.
Including low-pass filters, compression, etc.
"""

from matplotlib import pyplot as plt
import numpy as np
from scipy.signal import butter, lfilter, wiener
from filterpy.kalman import KalmanFilter

from constants import *

"""
Signal filters, especially low pass filters

"""


def apply_EMA_lastvalue(arr, alpha: float) -> float:
    """
    Calculate weighted sum, with newer values getting more weight
    apply EMA for the whole array, as array is already limited with length k
    this supports variable length for arr.
    """
    weights = np.array([(1 - alpha) ** (len(arr) - 1 - i) for i in range(len(arr))])
    weights /= weights.sum()  # Normalize weights
    weighted_average = np.dot(weights, np.array(arr))
    if DEBUG_PRINT:
        print(f">>>> in apply EMA:")
        print(f">> weights: {weights}")
        print(f">> arr: {arr}")
        print(f">> weighted_avg: {weighted_average}")
        print(f">>>>")
    return weighted_average


def apply_MA_lastvalue(arr) -> float:
    # just use simple moving average
    average = sum(arr) / len(arr)
    return average


def apply_Butterworth_lastvalue(arr, cutoff: float, fs: float, order: int = 2) -> float:
    """
    Applies a Butterworth low-pass filter to an entire array of values and
    returns the last filtered value.
    This filter is FIR since a window is used.

    Parameters:
    arr (array-like): Input array of signal values, with the most recent value last.
    cutoff (float): The cutoff frequency of the filter.
    fs (float): The sampling rate of the signal.
    order (int): The order of the Butterworth filter (higher = sharper cutoff).

    Returns:
    float: The last value of the filtered signal.
    """
    b, a = butter(order, cutoff / (0.5 * fs), btype="low", analog=False)
    filtered_arr = lfilter(b, a, arr)
    return filtered_arr[-1]


class ButterworthFilter:
    """
    class version.
    no control of windowed parameter k but efficient (supposed) as this is regressive
    """

    def __init__(self, order, cutoff, sample_rate):
        self.order = order
        self.cutoff = cutoff
        self.sample_rate = sample_rate
        self.b, self.a = butter(order, cutoff / (0.5 * sample_rate), btype="low")
        self.zi = np.zeros(max(len(self.a), len(self.b)) - 1)

    def __call__(self, new_value):
        filtered_value, self.zi = lfilter(self.b, self.a, [new_value], zi=self.zi)
        return filtered_value[0]


def apply_Wiener_lastvalue(arr, k_wiener: int, noise: float = None):
    """
    Applies a Wiener filter to the entire array and returns the last filtered value.
    This is pseudo FIR since window is provided.

    Parameters:
    arr (array-like): Input array of signal values.
    k_wiener (int): The size of the filter window.
    noise (float, optional): Noise power. If None, it will be estimated from the signal.

    Returns:
    float: The last value of the filtered signal.
    """
    if len(arr) > 1:
        filtered_arr = wiener(arr, mysize=k_wiener, noise=noise)
        return filtered_arr[-1]
    else:
        return 0


class WienerFilter:
    """
    class version of wiener filter.
    loses the control of k (from FIR to IIR) but more efficient as it is regressive (supposed)
    """

    def __init__(self, noise_estimate=0.1):
        self.signal_estimate = 0.0
        self.noise_estimate = noise_estimate
        self.alpha = 1.0  # Smoothing factor for adaptation

    def __call__(self, new_value):
        # Estimate signal power and noise power
        self.signal_estimate = (
            self.alpha * new_value**2 + (1 - self.alpha) * self.signal_estimate
        )
        snr_estimate = self.signal_estimate / (self.noise_estimate + 1e-9)
        # Compute Wiener gain
        wiener_gain = snr_estimate / (1 + snr_estimate)
        # Update the signal estimate and apply the Wiener filter
        filtered_value = wiener_gain * new_value
        return filtered_value


def apply_Kalman_lastvalue(arr, R=0.01, Q=1e-5):
    """
    Applies a Kalman filter to the entire array and returns the last filtered value.
    Since a windowed array can be provided, this can be pseudo-FIR.

    Parameters:
    arr (array-like): Input array of signal values.
    R (float): Measurement noise covariance.
    Q (float): Process noise covariance.

    Returns:
    float: The last value of the filtered signal.
    """
    # Initialize Kalman filter parameters
    kf = KalmanFilter(dim_x=1, dim_z=1)
    kf.x = np.array([[0.0]])  # Initial state (initial estimate)
    kf.F = np.array([[1.0]])  # State transition matrix
    kf.H = np.array([[1.0]])  # Measurement function
    kf.P = np.array([[1.0]])  # Covariance matrix (initial estimate error)
    kf.R = R  # Measurement noise
    kf.Q = Q  # Process noise

    # Apply the Kalman filter to each value in the array
    for value in arr:
        kf.predict()
        kf.update(value)

    # Return the last filtered value
    return kf.x[0, 0]


class OneDimensionalKalmanFilter:
    """
    class version of kalman filter.
    since this is regressive by nature, this is more efficient.
    """

    def __init__(
        self,
        process_variance,
        measurement_variance,
        initial_estimate=0.0,
        initial_estimate_error=1.0,
    ):
        # Initialize the Kalman filter for a one-dimensional system
        self.kf = KalmanFilter(dim_x=1, dim_z=1)

        # Define the initial state and uncertainty
        self.kf.x = np.array([[initial_estimate]])  # Initial estimate
        self.kf.P = np.array([[initial_estimate_error]])  # Initial estimate error

        # Define the state transition matrix
        self.kf.F = np.array([[1]])  # Constant state for this simple case

        # Define the measurement matrix
        self.kf.H = np.array([[1]])  # Direct measurement

        # Define process and measurement noise
        self.kf.R = np.array([[measurement_variance]])  # Measurement noise
        self.kf.Q = np.array([[process_variance]])  # Process noise

    def update(self, measurement):
        # Perform the prediction step
        self.kf.predict()

        # Perform the update step with the new measurement
        self.kf.update(np.array([[measurement]]))

        # Return the current state estimate
        return self.kf.x[0, 0]


"""
Signal processing pipes, or mixins
"""


class DynamicRangeCompressor:
    """
    Dynamic range compression, class version. Softknee and absolute value
    compression included.
    As it is iterative in nature, implemented as a class.
    """

    def __init__(
        self,
        threshold: float = 0.5,
        ratio: float = 3,
        attack: float = 0.01,
        release: float = 0.1,
        knee_width: float = 0.1,
        bias: float = 0,
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

    def compress(self, signal_value: float | np.ndarray) -> float:
        if not isinstance(signal_value, float | int):
            # if not a float or int, assume that it is a container and just get the latest one
            signal_value = signal_value[-1]
        """
        applies a soft knee.
        """
        abs_value = abs(signal_value - self.bias)

        # Check if we're in the knee region
        if abs_value > self.threshold + self.knee_width / 2:
            # Above knee region, apply full ratio
            excess = abs_value - self.threshold
            target_gain_reduction = (
                np.sign(signal_value) * excess * (1 - 1 / self.ratio)
            )
        elif abs_value > self.threshold - self.knee_width / 2:
            # Within the knee region, apply gradual compression
            # Calculate how far into the knee we are as a percentage
            distance_into_knee = (
                abs_value - (self.threshold - self.knee_width / 2)
            ) / self.knee_width
            # Interpolate the ratio based on the distance into the knee
            effective_ratio = 1 + (self.ratio - 1) * distance_into_knee**2
            excess = abs_value - self.threshold + self.knee_width / 2
            target_gain_reduction = (
                np.sign(signal_value) * excess * (1 - 1 / effective_ratio)
            )
        else:
            # Below knee region, no compression
            target_gain_reduction = 0

        # Smoothly adjust gain reduction toward the target using attack/release
        if self.gain_reduction < target_gain_reduction:
            self.gain_reduction += self.attack
            if self.gain_reduction > target_gain_reduction:
                self.gain_reduction = target_gain_reduction
        elif self.gain_reduction > target_gain_reduction:
            self.gain_reduction -= self.release
            if self.gain_reduction < target_gain_reduction:
                self.gain_reduction = target_gain_reduction

        # Apply the smoothed gain reduction to the signal value
        compressed_value = signal_value - self.gain_reduction

        return compressed_value

    def reset(self):
        self.gain_reduction = 0


"""
utility to drive filters to work
"""


"""
test functions

"""


def test_dynamic_range_compressor():
    # Parameters for the sine wave
    sample_rate = 1000  # Samples per second
    duration = 2  # Duration in seconds
    frequency = 5  # Frequency of the sine wave in Hz
    time = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    sine_wave = np.sin(2 * np.pi * frequency * time)

    # Add noise and spikes to the sine wave
    noise = np.random.normal(0, 0.1, sine_wave.shape)  # Gaussian noise
    spikes = np.zeros_like(sine_wave)
    spike_indices = np.random.choice(len(sine_wave), size=10, replace=False)
    spikes[spike_indices] = np.random.uniform(-1, 1, size=10)  # Random spikes
    noisy_sine_wave = sine_wave + noise + spikes

    # Create an instance of the DynamicRangeCompressor
    compressor = DynamicRangeCompressor(
        threshold=0.5, ratio=4, attack=0.01, release=0.05, knee_width=0.2
    )

    # Apply compression to the noisy sine wave
    compressed_wave = []
    for sample in noisy_sine_wave:
        compressed_sample = compressor.compress(sample)
        compressed_wave.append(compressed_sample)
    compressed_wave = np.array(compressed_wave)

    # Plot the original and compressed sine wave
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


# Test the DynamicRangeCompressor with a sine wave
if __name__ == "__main__":
    test_dynamic_range_compressor()
