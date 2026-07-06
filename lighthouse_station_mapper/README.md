# Lighthouse Station Mapper

An interactive terminal-based tool for calibrating and mapping Lighthouse base station positions and orientations in 3D space.

## Overview

This package provides a ROS 2 node with a terminal UI (built on [FTXUI](https://github.com/ArthurSonzogni/ftxui)) that guides you through the process of determining the precise geometry of your Lighthouse tracking system. The node subscribes to raw lighthouse measurements from a deck and uses optimization to solve for the station poses.

## Quick Start

Launch the interactive mapper with a connected lighthouse deck:

```bash
ros2 launch lighthouse_station_mapper interactive_mapping.launch.py device:=/dev/ttyACM0
```

See example in this video:

https://github.com/user-attachments/assets/1d2d9ecf-f871-4dcb-8961-9a4889b39a24

## Mapping Workflow

1. **Position the deck** in your workspace where lighthouse stations are visible
2. **Wait for "READY TO SAMPLE"** status - the system needs stable measurements from all visible stations
3. **Press 'Sample' (or 's')** to capture a measurement snapshot at this position
   - The first sample you take defines the origin of the coordinate system
4. **Move the deck to a different position** (at least 10-20 cm away with some rotation)
5. **Repeat steps 2-4** to collect samples from multiple deck positions (minimum 2, more is better)
6. **Press 'Solve station poses' (or 'v')** to compute station poses
   - The coordinate system origin is placed at the first sample position
   - Results appear in the "Solution" table and are visualized in RViz (if enabled)
7. **Press 'Update map node'** to send the computed station poses to the localization node
   - This enables real-time pose tracking using the calibrated station geometry

### Other Controls

- **Save map** - Write the current station geometry to a CSV file
- **Clear samples** (or 'c') - Delete all samples and reset the solution
- **Quit** (or 'q') - Exit the mapper

## Published Topics

- **`mapper_station_markers`** (`visualization_msgs/msg/MarkerArray`)
  - RViz visualization markers showing the solved base station poses
  - Published with transient_local QoS for late-joining subscribers

- **`mapper_deck_pose_samples`** (`visualization_msgs/msg/MarkerArray`)
  - RViz visualization markers showing the deck positions where samples were captured
  - Helps visualize the spatial distribution of calibration data

- **`mapper_deck_pose`** (`geometry_msgs/msg/PoseStamped`)
  - Current estimated deck pose during the sampling process
  - Published at 10 Hz when the deck is in view of stations

## Subscribed Topics

- **`lighthouse`** (`lighthouse_deck_msgs/msg/LighthouseDeckMeasurement`)
  - Raw angle measurements from the lighthouse deck driver
  - Used to collect calibration samples and compute station geometry

## Service Clients

- **`set_station_poses`** (`lighthouse_station_mapper_msgs/srv/SetStationPoses`)
  - Called when the "Update map node" button is pressed
  - Sends the solved station poses to the lighthouse_localization node

## Parameters

- **`max_angular_spread`** (double, default: `5e-3`)
  - Maximum angular spread (in radians) allowed for measurements to be considered stable
  - Lower values require more stable/static deck positioning before sampling
  - Used to determine "READY TO SAMPLE" status

- **`buffer_duration`** (double, default: `1.0`)
  - Duration (in seconds) to buffer incoming measurements
  - Longer buffers provide more data for stability checks but increase latency

- **`min_samples_per_station`** (int, default: `3`)
  - Minimum number of deck pose samples required per station before solving is possible
  - More samples generally improve calibration accuracy

- **`lighthouse_frame`** (string, default: `"lighthouse"`)
  - Frame ID for the lighthouse coordinate system
  - Used in visualization markers and TF broadcasts
