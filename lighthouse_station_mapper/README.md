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

1. **Position the deck** in your workspace where lighthouse stations are visible
2. **Wait for "READY TO SAMPLE"** status - the system needs stable measurements from all visible stations
3. **Press 'Sample' (or 's')** to capture a measurement snapshot at this position
   - The first sample you take defines the origin of the coordinate system
4. **Move the deck to a different position** (at least 10-20 cm away with some rotation)
5. **Repeat steps 2-4** to collect samples from multiple deck positions (minimum 2, more is better)
6. **Press 'Solve' (or 'v')** to compute station poses
   - The coordinate system origin is placed at the first sample position
   - Results appear in the "Solution" table and are visualized in RViz (if enabled)

### Other Controls

- **Save map** (or 'w') - Write the current station geometry to a CSV file
- **Update map node** (or 'u') - Send current station poses to the localization node
- **Clear samples** (or 'c') - Delete all samples and reset the solution
- **Quit** (or 'q') - Exit the mapper
