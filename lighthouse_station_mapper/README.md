# Lighthouse Station Mapper

An interactive terminal-based tool for calibrating and mapping Lighthouse base station positions and orientations in 3D space.

## Overview

This package provides a ROS 2 node with a terminal UI (built on [FTXUI](https://github.com/ArthurSonzogni/ftxui)) that guides you through the process of determining the precise geometry of your Lighthouse tracking system. The node subscribes to raw lighthouse measurements from a deck and uses optimization to solve for the station poses.

## Quick Start

Launch the interactive mapper with a connected lighthouse deck:

```bash
ros2 launch lighthouse_station_mapper interactive_mapping.launch.py device:=/dev/ttyACM0
```

Optional arguments:
- `baudrate:=230400` - Serial port baudrate (default: 230400)
- `use_rviz:=true` - Launch RViz for visualization (default: false)

The mapper UI will open in a separate xterm window.

## Mapping Workflow

### Basic Workflow (Station-Based Origin)

1. **Position the deck** in your workspace where lighthouse stations are visible
2. **Wait for "READY TO SAMPLE"** status - the system needs stable measurements from all visible stations
3. **Press 'Sample' (or 's')** to capture a measurement snapshot at this position
4. **Move the deck to a different position** (at least 10-20 cm away with some rotation)
5. **Repeat steps 2-4** to collect samples from multiple deck positions (minimum 2, more is better)
6. **Press 'Solve (origin @station)' (or 'v')** to compute station poses
   - This places the first detected station at the coordinate system origin
   - Results appear in the "Solution" table and are visualized in RViz (if enabled)

### Advanced Workflow (Keypoint-Based Origin)

To define a custom coordinate frame using physical reference points:

1. **Complete the basic workflow** first to get an initial solution
2. **Position the deck at your desired origin point**
3. **Press 'Set origin keypoint' (or 'k')** - this records keypoint #1 (origin)
4. **Move the deck to a point along your desired +X axis**
5. **Press 'Set origin keypoint'** again - this records keypoint #2 (+X direction)
6. **Move the deck to a point that defines the XY plane** (should not be collinear with the first two)
7. **Press 'Set origin keypoint'** a third time - this records keypoint #3 (defines XY plane)
8. **Press 'Solve (origin @keypoint)' (or 'y')** to recompute station poses using your custom frame
   - The origin is now at keypoint #1
   - The +X axis points toward keypoint #2
   - The +Z axis is perpendicular to the plane defined by the three keypoints

### Other Controls

- **Save map** - Write the current station geometry to disk (not yet implemented)
- **Clear samples** (or 'c') - Delete all samples and reset the solution
- **Clear origin keypoints** (or 'r') - Remove all keypoints to start over with custom frame definition
- **Quit** (or 'q') - Exit the mapper
