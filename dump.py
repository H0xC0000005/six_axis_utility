import numpy as np
import matplotlib.pyplot as plt


class DynamicRangeCompressor:
    def __init__(
        self, threshold=0.5, ratio=4, attack=0.01, release=0.1, knee_width=0.1
    ):
        self.threshold = threshold
        self.ratio = ratio
        self.attack = attack
        self.release = release
        self.knee_width = knee_width
        self.gain_reduction = (
            0  # Internal state to remember gain reduction between timesteps
        )

    def process(self, signal_value):
        abs_value = abs(signal_value)

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


# Example usage remains the same
compressor = DynamicRangeCompressor(threshold=0.5, ratio=4, attack=0.01, release=0.1)

# Simulated real-time signal input
time = np.linspace(0, 1, 1000)
original_signal = np.sin(2 * np.pi * 5 * time) * 0.8 + np.random.randn(1000) * 0.05
original_signal[100:150] += 0.5  # Adding a sudden peak for testing
original_signal[200:250] -= 0.5  # Adding a sudden negative peak for testing

# Process each value in the signal one at a time
compressed_signal = np.zeros_like(original_signal)
for i in range(len(original_signal)):
    compressed_signal[i] = compressor.process(original_signal[i])

# Plot the result
plt.figure(figsize=(12, 6))
plt.plot(time, original_signal, label="Original Signal")
plt.plot(time, compressed_signal, label="Compressed Signal", linewidth=2)
plt.xlabel("Time")
plt.ylabel("Amplitude")
plt.legend()
plt.title("Adaptive Dynamic Range Compression Example")
plt.show()
