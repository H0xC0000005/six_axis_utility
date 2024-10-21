# Six Axis Utility

This repository contains the driver for the Six Axis platform on the application side.

## Overview

The SDK used to directly drive the platform should be installed separately. This utility allows you to configure, control, and manage the Six Axis platform effectively.

## Configuration Specifications

The equalizer uses a YAML config file to equalize the dimensions. The parameters of the equalizer are specified as follows:

- **enable** (`bool`): Specifies if this dimension is enabled.

- **FilterMode** (`dict`): Defines the filtering mode for this dimension. This is a nested item with the following parameters:
  - **mode** (`string`): The filter applied. If not specified, the default identity filter is used.
  - **k** (`int`): The filter window. This parameter adjusts the granularity and affects the filter behavior, creating a pseudo-FIR response.

- **filter parameters**: The parameters depend on the filter type. Please refer to `constants.py` for a complete definition of the required variables.

- **gain** (`float`): Specifies how much gain is applied to this channel. This is applied at the very last step of the equalizer pipeline.

## Installation

To install this utility, clone the repository and install any necessary dependencies:

```sh
# Clone the repository
git clone <repository-url>

# Change directory
cd six_axis_utility

# Install dependencies
pip install -r requirements.txt
