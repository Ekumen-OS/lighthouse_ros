#!/usr/bin/python3

# Copyright 2023 Gerardo Puga
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
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg_share = FindPackageShare("lighthouse_testbench_bringup")

    rviz_config_path = PathJoinSubstitution(
        [
            this_pkg_share,
            "rviz",
            "model.rviz",
        ]
    )

    station_poses_path = PathJoinSubstitution(
        [
            this_pkg_share,
            "config",
            "configuration.yaml",
        ]
    )

    lighthouse_map_server_node = Node(
        package="lighthouse_ros",
        executable="lighthouse_map_server",
        output="screen",
        parameters=[
            {
                "station_poses_path": station_poses_path,
            }
        ],
    )

    lighthouse_testbench_node = Node(
        package="lighthouse_testbench",
        executable="lighthouse_testbench",
        output="screen",
        parameters=[
            {
                "sensor_height": 0.0,
                "station_poses_path": station_poses_path,
            }
        ],
    )

    lighthouse_amcl_node = Node(
        package="lighthouse_amcl",
        executable="lighthouse_amcl_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        respawn=True,
        parameters=[],
        prefix="xterm -e",
        remappings=[
            ("landmarks_map", "/landmarks/landmarks_map"),
            ("landmark_detections", "/landmarks/landmark_detections"),
        ],
    )

    teleop_twist_keyb_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        respawn=True,
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [
            lighthouse_testbench_node,
            lighthouse_map_server_node,
            lighthouse_amcl_node,
            teleop_twist_keyb_node,
            rviz_node,
        ]
    )
