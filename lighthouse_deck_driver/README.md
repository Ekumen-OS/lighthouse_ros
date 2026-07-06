# Lighthouse Deck Driver

ROS 2 driver node that interfaces with the Lighthouse Positioning Deck via USB-to-serial adapter.

## Overview

This node communicates with the Lighthouse Positioning Deck hardware through a serial connection, decodes the binary protocol, and publishes raw angle measurements from the lighthouse sensors. It handles the deck's bootloader initialization sequence and configures the serial communication at the appropriate baudrate.

## Published Topics

- **`lighthouse`** (`lighthouse_deck_msgs/msg/LighthouseDeckMeasurement`)
  - Raw azimuth and elevation angle measurements from all four sensors on the deck for each visible base station
  - Published whenever a complete sweep from a base station is received
  - Message includes station ID and eight angle values (azimuth and elevation for each of the four sensors)

## Subscribed Topics

None.

## Parameters

- **`device`** (string, **required**)
  - Serial device path for the Lighthouse Deck (e.g., `/dev/ttyUSB0`, `/dev/ttyACM0`)
  - No default value; must be specified when launching the node

- **`baudrate`** (int, default: `230400`)
  - Baudrate for serial communication with the deck's main firmware
  - Valid values: `9600`, `19200`, `38400`, `57600`, `115200`, `230400`
  - The node automatically handles bootloader communication at 115200 before switching to this baudrate

- **`frame_id`** (string, default: `"lighthouse_deck"`)
  - Frame ID to use in the header of published measurement messages

## Example Usage

```bash
# Launch with default baudrate
ros2 run lighthouse_deck_driver lighthouse_deck_driver_node --ros-args -p device:=/dev/ttyACM0

# Launch with custom baudrate and frame ID
ros2 run lighthouse_deck_driver lighthouse_deck_driver_node --ros-args \
  -p device:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p frame_id:=my_lighthouse_deck
```
