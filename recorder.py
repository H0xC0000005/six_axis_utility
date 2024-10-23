import time
import pandas as pd
import numpy as np
from typing import Iterable
import yaml
from constants import *


class Recorder:

    def __init__(self, config_path=None):
        self.data_list = []
        config = None
        if config_path is not None:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
        self.config = config

    def update_record(self, record):
        self.data_list.append(record)

    def to_csv(self, destination_file, dims=None):
        if not destination_file.endswith(".csv"):
            raise ValueError(
                f"destination file must be a .csv file. got {destination_file}"
            )
        df = pd.DataFrame(self.data_list)
        print(df.head())
        if dims is None and self.config is not None:
            dims = [k for k, v in self.config.items() if v]
        print(dims)
        if dims is not None:
            df = df[[TIMESTAMP_NAME] + list(dims)]
        df.to_csv(destination_file, index=False)


def test_recorder():
    SIGNAL_1 = "Yaw"
    SIGNAL_2 = "Pitch"
    SIGNAL_3 = "Roll"
    SIGNAL_4 = "Sway"
    SIGNAL_5 = "Heave"
    SIGNAL_6 = "Surge"
    config_path = "./configs/recorder_config.yaml"
    recorder = Recorder(config_path=config_path)
    for i in range(10):
        current_time = time.time()
        signal_1_value = np.sin(i)
        signal_2_value = np.cos(i)
        signal_3_value = np.sin(i + 0.5)
        signal_4_value = np.cos(i + 0.5)
        signal_5_value = np.sin(i + 1)
        signal_6_value = np.cos(i + 1)
        record = {
            TIMESTAMP_NAME: current_time,
            SIGNAL_1: signal_1_value,
            SIGNAL_2: signal_2_value,
            SIGNAL_3: signal_3_value,
            SIGNAL_4: signal_4_value,
            SIGNAL_5: signal_5_value,
            SIGNAL_6: signal_6_value,
        }
        recorder.update_record(record)
        time.sleep(0.05)
    recorder.to_csv("dummy_signals.csv")
    print("Dummy signals have been recorded and saved to dummy_signals.csv")


if __name__ == "__main__":
    test_recorder()
