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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz_arg, use_rviz_conf = (
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Whether to launch RViz2 for visualization",
        ),
        LaunchConfiguration("use_rviz"),
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("lighthouse_station_mapper"), "rviz", "config.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz_conf),
    )

    mapper_node = Node(
        package="lighthouse_station_mapper",
        namespace="",
        output="screen",
        executable="mapper_ui",
        arguments=["--ros-args", "--log-level", "INFO"],
        prefix="xterm -geometry 160x50 -fa 'Monospace' -fs 12 -e",
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
            use_rviz_arg,
            mapper_node,
            rviz_node,
            shutdown_on_mapper_exit,
        ]
    )
