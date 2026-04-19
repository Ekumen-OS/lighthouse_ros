# Copyright 2026 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device_arg, device_conf = (
        DeclareLaunchArgument(
            "device",
            description="Serial device to connect to",
        ),
        LaunchConfiguration("device"),
    )

    baudrate_arg, baudrate_conf = (
        DeclareLaunchArgument(
            "baudrate",
            default_value="230400",
            description="Baudrate to use for the serial connection",
        ),
        LaunchConfiguration("baudrate"),
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("lighthouse_station_mapper"), "rviz", "config.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    lighthouse_driver_node = Node(
        package="lighthouse_deck_driver",
        namespace="",
        output="screen",
        executable="lighthouse_deck_driver",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[
            {
                "device": device_conf,
                "baudrate": baudrate_conf,
            }
        ],
    )

    mapper_node = Node(
        package="lighthouse_station_mapper",
        namespace="",
        output="screen",
        executable="mapper_ui",
        arguments=["--ros-args", "--log-level", "INFO"],
        prefix="xterm -e",
    )

    # Shutdown launch when mapper node exits
    shutdown_on_mapper_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=mapper_node,
            on_exit=[Shutdown(reason="Mapper node exited")],
        )
    )

    return LaunchDescription(
        [
            device_arg,
            baudrate_arg,
            lighthouse_driver_node,
            mapper_node,
            rviz_node,
            shutdown_on_mapper_exit,
        ]
    )
