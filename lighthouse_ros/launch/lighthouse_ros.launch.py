# Copyright 2024 Ekumen, Inc.
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
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


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

    lighthouse_ros_node = Node(
        package="lighthouse_ros",
        namespace="",
        output="screen",
        executable="lighthouse_ros",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[
            {
                "device": device_conf,
                "baudrate": baudrate_conf,
            }
        ],
    )

    return LaunchDescription(
        [
            device_arg,
            baudrate_arg,
            lighthouse_ros_node,
        ]
    )
