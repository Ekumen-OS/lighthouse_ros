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

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Arguments for live mode (from serial device)
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="/dev/ttyUSB0",
        description="Serial device to connect to",
    )
    device_conf = LaunchConfiguration("device")

    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="230400",
        description="Baudrate to use for the serial connection",
    )
    baudrate_conf = LaunchConfiguration("baudrate")

    # Flag to set live or test mode
    run_testbench_arg = DeclareLaunchArgument(
        'run_testbench',
        default_value='False',
        description='Whether to use the testbench to simulate data.',
    )
    run_testbench = LaunchConfiguration('run_testbench')

    # Group for live mode
    live_group = GroupAction(
        condition=UnlessCondition(run_testbench),
        actions=[
            Node(
                package="lighthouse_ros",
                executable="lighthouse_ros",
                output="screen",
                arguments=["--ros-args", "--log-level", "INFO"],
                parameters=[{"device": device_conf, "baudrate": baudrate_conf}],
            )
        ],
    )

    # Group for test mode
    included_package_dir = get_package_share_directory('lighthouse_testbench_bringup')
    included_launch_file_path = os.path.join(
        included_package_dir, 'launch', 'main.launch.py'
    )

    testbench_group = GroupAction(
        condition=IfCondition(run_testbench),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(included_launch_file_path)
            )
        ],
    )

    # Estimation node, running on both live and test mode
    base_geometry_estimation_node = Node(
        package='lighthouse_base_geometry_estimation',
        executable='base_geometry_estimation',
        name='base_geometry_estimation_node',
        output='screen',
    )

    return LaunchDescription(
        [
            device_arg,
            baudrate_arg,
            run_testbench_arg,
            base_geometry_estimation_node,
            live_group,
            testbench_group,
        ]
    )
