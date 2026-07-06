# Lighthouse Localization

ROS 2 node that computes real-time 6-DOF pose estimates of the Lighthouse Deck using calibrated base station geometry.

## Overview

This node performs real-time localization by solving for the deck's position and orientation given raw lighthouse angle measurements and known base station poses. It uses nonlinear optimization to find the deck pose that best explains the observed sensor angles. The node requires station poses to be configured (either via service call or CSV file) before it can compute poses.

## Published Topics

- **`deck_pose`** (`geometry_msgs/msg/PoseStamped`)
  - Current estimated 6-DOF pose of the lighthouse deck in the map frame
  - Published at up to `max_solver_rate` Hz when sufficient measurements are available
  - Pose represents the transformation from the deck frame to the map frame

## Subscribed Topics

- **`lighthouse`** (`lighthouse_deck_msgs/msg/LighthouseDeckMeasurement`)
  - Raw angle measurements from the lighthouse deck driver

## Services

- **`set_station_poses`** (`lighthouse_station_mapper_msgs/srv/SetStationPoses`)
  - Service to configure the base station poses for localization
  - Must be called before the node can publish pose estimates if no station map file is provided. Provided for the lighthouse_station_mapper after solving station geometry.

## Parameters

- **`stations_map`** (string, default: `""`)
  - Path to a CSV file containing pre-calibrated station poses
  - If provided, station poses are loaded automatically at startup
  - File format should match the output of lighthouse_station_mapper

- **`map_frame`** (string, default: `"lighthouse"`)
  - Frame ID for the map/world coordinate system
  - Used in the header of published pose messages

- **`time_tolerance`** (double, default: `0.1`)
  - Maximum time window for sample synchronization (seconds)
  - Measurements older than this are discarded from the buffer
  - Default is approximately two cycles at 50Hz tracking rate

- **`max_solver_rate`** (double, default: `30.0`)
  - Maximum rate at which pose optimization is executed (Hz)
  - Limits computational load while maintaining responsive tracking
  - Actual publish rate may be lower if measurements arrive slowly

## Example Usage

```bash
# Launch with station poses loaded from file
ros2 run lighthouse_localization lighthouse_localization_node --ros-args \
  -p stations_map:=/path/to/stations.csv

# Launch with custom parameters
ros2 run lighthouse_localization lighthouse_localization_node --ros-args \
  -p map_frame:=world \
  -p max_solver_rate:=50.0 \
  -p time_tolerance:=0.05
```
