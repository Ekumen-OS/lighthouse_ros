# Lighthouse ROS 2

This repository contains a ROS 2 node that decodes the output serial stream from a `lighthouse deck` and publishes the azimuth and elevation angles data to ROS messages. This allows to interface ROS 2 systems with the LightHouse positioning system for the Crazyflie 2.1.

## TL;DR instructions for people already up to speed

Build and run the development docker container:

```bash
./docker/run.sh --build
```

Once within the docker, build the ROS 2 workspace:

```bash
colcon build --symlink-install
. install/setup.bash
```

The node takes arguments for the serial port and the baudrate, so the best way to run it is with a launch file. A helper launch file is provided in the `launch` directory for node testing purposes:

```bash
ros2 launch lighthouse_ros lighthouse_ros.launch.py device:=/dev/ttyACM0 baudrate:=115200
```

The `baudrate` argument is optional and defaults to 230400. The `device` argument is mandatory.

## TL;DR guide to do a test run using the datasets in the repository

There are two ways to test the code.

#### Offline protocol parser node

The decoding stack can be tested using a second executable provided in the same package. This executable reads the data stream from a file and plays it through the same decoding stack as the main ROS 2 node.

This will only produce logging output, and is therefore useful to test the protocol parsing code offline.

To play a file through the decoding stack in this way, run the following command:

```bash
ros2 run lighthouse_ros file_parser_node <path_to_file>
```

For example:

```bash
ros2 run lighthouse_ros file_parser_node src/lighthouse_ros_ros2_stack/datasets/raw_frames.txt
```

See `file_parser_node.py` for more information.

#### Full run of the ROS 2 node

To be able to replay a recorded dataset through the main node you need to create "virtual" serial port on your system that the node can connect. Open a new terminal in your development docker container and run the following command:

```bash
socat -d -d pty,link=/tmp/vserial1,raw,echo=0 pty,link=/tmp/vserial2,raw,echo=0
```

This will create a pair of pseudo terminals, `/tmp/vserial1` and `/tmp/vserial2`.
You can then connect the node to `/tmp/vserial1` from a second terminal using the
helper launch file as:

```bash
ros2 launch lighthouse_ros lighthouse_ros.launch.py device:=/tmp/vserial1
```

Finally, you simply dump the dataset to the other end of the pseudo terminal pair:
```bash
cat src/lighthouse_ros_ros2_stack/datasets/raw_frames.txt > /tmp/vserial2
```

You can watch the output messages using `ros2 topic echo` or `ros2 topic hz` on the `/lighthouse` topic.
```bash
ros2 topic echo /lighthouse
```

## References

- [Lighthouse Positioning System: Dataset, Accuracy, and Precision for UAV Research](https://arxiv.org/abs/2104.11523), paper with description of the
 overall system.
- [Repurposing Valve's SteamVR 2.0 Technology to Develop an Open-Source, Low-Cost Motion Capture System for Robotics](https://fosdem.org/2025/schedule/event/fosdem-2025-5013-repurposing-valve-s-steamvr-2-0-technology-to-develop-an-open-source-low-cost-motion-capture-system-for-robotics/), great talk at FOSDEM 2025 about the system.
- [Lighthouse deck bootloader verilog code repository](https://github.com/bitcraze/lighthouse-bootloader), including the protocol to interact with the bootloader and make it boot the main firmware.
- [Lighthouse deck firmware repository](https://github.com/bitcraze/lighthouse-fpga), including a general description of the protocol used to communicate with the deck.
