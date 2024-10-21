import socket
import struct
import math
import time
import threading
import signal
import sys
from typing import Tuple

import ipaddress

from hyperparams import Hyperparams
from constants import *

IPEndpoint = Tuple[str, int]


class GameStatus:
    GameOver = 0
    GameStart = 1


class SpecialEffect:
    NoneEffect = 0
    First = 1
    Second = 2
    Third = 4
    Fourth = 8
    Fifth = 16
    Sixth = 32
    Seventh = 64
    Eighth = 128
    Ninth = 256
    Tenth = 512
    Eleventh = 1024
    Twelfth = 2048


class CustomGamePack:

    pack_format = "<Iffffffl"

    def __init__(self):
        self.GameStatus = GameStatus.GameOver
        self.Roll = 0.0
        self.Yaw = 0.0
        self.Pitch = 0.0
        self.Surge = 0.0
        self.Heave = 0.0
        self.Sway = 0.0
        self.SpecialEffect = SpecialEffect.NoneEffect

    def to_bytes(self) -> bytes:
        return struct.pack(
            CustomGamePack.pack_format,
            self.GameStatus,
            self.Roll,
            self.Yaw,
            self.Pitch,
            self.Surge,
            self.Heave,
            self.Sway,
            self.SpecialEffect,
        )

    @staticmethod
    def from_bytes(byte_data: bytes):
        unpacked_data = struct.unpack(CustomGamePack.pack_format, byte_data)
        game_pack = CustomGamePack()
        game_pack.GameStatus = unpacked_data[0]
        game_pack.Roll = unpacked_data[1]
        game_pack.Yaw = unpacked_data[2]
        game_pack.Pitch = unpacked_data[3]
        game_pack.Surge = unpacked_data[4]
        game_pack.Heave = unpacked_data[5]
        game_pack.Sway = unpacked_data[6]
        game_pack.SpecialEffect = unpacked_data[7]
        return game_pack

    def print_stats(self):
        print("----------------------------------------------------------")
        print("current pack stats:")
        print(
            f"status: {self.GameStatus}, \troll: {self.Roll}, \tyaw: {self.Yaw}, \tpitch: {self.Pitch}"
        )
        print(
            f"surge: {self.Surge}, \theave: {self.Heave}, \tSway: {self.Sway}, \teffect: {self.SpecialEffect}"
        )
        print("----------------------------------------------------------")

    @classmethod
    def from_dict(cls, dict_val: dict[str, float]) -> "CustomGamePack":
        result = cls()
        result.Surge = dict_val[SURGE_NAME]
        result.Heave = dict_val[HEAVE_NAME]
        result.Sway = dict_val[SWAY_NAME]
        result.Yaw = dict_val[YAW_NAME]
        result.Pitch = dict_val[PITCH_NAME]
        result.Roll = dict_val[ROLL_NAME]
        return result


class ControlSender:
    def __init__(self):
        self.status_count = 0

    def generate_next_pack_orientation_only(self):
        print(f"Generating next pack. Status count: {self.status_count}")
        time.sleep(Hyperparams.interval)
        result = CustomGamePack()

        phase_period = Hyperparams.period // 3
        phase = self.status_count % phase_period
        cur_value = math.sin(phase / phase_period * 2 * math.pi) * Hyperparams.amplitude

        if self.status_count // phase_period < 1:
            result.Roll = cur_value
        elif self.status_count // phase_period < 2:
            result.Yaw = cur_value
        elif self.status_count // phase_period < 3:
            result.Pitch = cur_value
        else:
            print("Finished a cycle. Resetting status")
            self.status_count = 0

        result.GameStatus = GameStatus.GameStart
        return result

    def generate_next_pack(self):
        print(f"Generating next pack. Status count: {self.status_count}")
        time.sleep(Hyperparams.interval)
        result = CustomGamePack()

        phase_period = Hyperparams.period // 6
        phase = self.status_count % phase_period
        cur_value = math.sin(phase / phase_period * 2 * math.pi) * Hyperparams.amplitude

        # WARNING: units of pitch/roll/yaw is percentage of piston length.
        # units of surge/heave/sway is much larger, so be careful!
        if self.status_count // phase_period < 1:
            result.Roll = cur_value
        elif self.status_count // phase_period < 2:
            result.Yaw = cur_value
        elif self.status_count // phase_period < 3:
            result.Pitch = cur_value
        elif self.status_count // phase_period < 4:
            result.Surge = cur_value
        elif self.status_count // phase_period < 5:
            result.Heave = cur_value
        elif self.status_count // phase_period < 6:
            result.Sway = cur_value
        else:
            print("Finished a cycle. Resetting status")
            self.status_count = 0

        result.GameStatus = GameStatus.GameStart
        return result

    def test_roll(self):
        print(f"Generating next pack (test roll). Status count: {self.status_count}")
        time.sleep(Hyperparams.interval)
        result = CustomGamePack()
        result.Roll = Hyperparams.amplitude
        result.GameStatus = GameStatus.GameStart
        return result

    def test_single_dim_sinusodial(self, dim: str):
        rollname, pitchname, yawname, surgename, swayname, heavename = (
            "Roll",
            "Pitch",
            "Yaw",
            "Surge",
            "Sway",
            "Heave",
        )
        valid_dims = (rollname, pitchname, yawname, surgename, swayname, heavename)
        if dim not in valid_dims:
            raise ValueError(f"invalid dim provided. must be one of {valid_dims}")
        print(f"Generating next pack (test heave). Status count: {self.status_count}")
        time.sleep(Hyperparams.interval)
        result = CustomGamePack()
        phase_period = Hyperparams.period // 6
        phase = self.status_count % phase_period
        cur_value = math.sin(phase / phase_period * 2 * math.pi) * Hyperparams.amplitude

        if dim == rollname:
            result.Roll = cur_value
        elif dim == pitchname:
            result.Pitch = cur_value
        elif dim == yawname:
            result.Yaw = cur_value
        elif dim == surgename:
            result.Surge = cur_value * 0.25
        elif dim == swayname:
            result.Sway = cur_value * 0.25
        elif dim == heavename:
            result.Heave = cur_value * 0.25
        result.GameStatus = GameStatus.GameStart
        return result


def send_pack(udp_client: socket.socket, end_point: IPEndpoint, pack: CustomGamePack):
    pack_serialized = pack.to_bytes()
    udp_client.sendto(pack_serialized, end_point)
    pack.print_stats()


def on_exit(
    sig, frame, udp_client: socket.socket, end_point: IPEndpoint, control_sender
):
    print("Keyboard interrupt. Sending GameOver packs")
    control_sender.status_count = 0  # Stop the main loop
    end_pack = CustomGamePack()
    for _ in range(10):
        print("Sending end pack")
        send_pack(udp_client, end_point, end_pack)
        time.sleep(Hyperparams.interval)
    udp_client.close()
    sys.exit(0)


def main():
    control_sender = ControlSender()
    ip_address = input(
        "Enter the destination IP address (e.g., 127.0.0.1, 192.168.0.208): "
    )
    try:
        ipaddress.IPv4Address(ip_address)
    except ValueError:
        print(
            f"entered an invalid ip address: |{ip_address}|. using default 192.168.0.208"
        )
        print(f"i.e. six axis platform pc.")
        ip_address = "192.168.0.208"
    port = 15620
    end_point = (ip_address, port)

    udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Set up the signal handler for graceful shutdown
    signal.signal(
        signal.SIGINT,
        lambda sig, frame: on_exit(sig, frame, udp_client, end_point, control_sender),
    )

    try:
        while True:
            # TODO: change this line to test with different patterns
            # -
            # current_pack = control_sender.generate_next_pack()
            current_pack = control_sender.generate_next_pack()
            send_pack(udp_client, end_point, current_pack)
            control_sender.status_count += 1
    except KeyboardInterrupt:
        on_exit(None, None, udp_client, end_point, control_sender)


if __name__ == "__main__":
    main()
