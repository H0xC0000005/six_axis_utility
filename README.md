# Six Axis Utility

This repository contains the driver for the Six Axis platform on the application side.

## Overview

The SDK used to directly drive the platform should be installed separately. This utility allows you to configure, control, and manage the Six Axis platform effectively.
there are two versions of this adapter. carla_to_six_axis_adapter.py uses rclpy (ros2 python API), and carla_to_six_axis_adapter_zmq.py uses zmq.

## Configuration Specifications

The equalizer uses a YAML config file to equalize the dimensions. The parameters of the equalizer are specified as follows:

- **enable** (`bool`): Specifies if this dimension is enabled.

- **FilterMode** (`dict`): Defines the filtering mode for this dimension. This is a nested item with the following parameters:
  - **mode** (`string`): The filter applied. If not specified, the default identity filter is used.
  - **k** (`int`): The filter window. This parameter adjusts the granularity and affects the filter behavior, creating a pseudo-FIR response.

- **filter parameters**: The parameters depend on the filter type. Please refer to `constants.py` for a complete definition of the required variables.

- **gain** (`float`): Specifies how much gain is applied to this channel. This is applied at the very last step of the equalizer pipeline.

you can switch which configuration to use in the adapter.

you should also check constants.py, and modify IMU_RANGES based on extreme values of your own data source.
you should also write your own handler in utilities.py, in section "processing raw imu dict handlers". the adapter uses EAFP style and a chain of responsibility so your handler will be called if it is the first handler that handles the data without exception.

## Installation

To install this utility, clone the repository and install any necessary dependencies:

```sh
# Clone the repository
git clone <repository-url>

# Change directory
cd six_axis_utility

# Install dependencies
pip install -r requirements.txt
