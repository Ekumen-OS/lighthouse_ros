# Copyright 2025 Ekumen
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
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='image_raw', description='Output image topic name'
    )

    # Load camera configuration
    camera_config_file = PathJoinSubstitution(
        [FindPackageShare('lighthouse_oak_camera'), 'config', 'oak_lr_basic_params.yaml']
    )

    # Simple OAK-LR Camera Driver Node (single camera mode)
    camera_node = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        parameters=[
            camera_config_file,
        ],
        remappings=[
            ('mono/image_raw', LaunchConfiguration('image_topic')),
            ('mono/camera_info', 'camera_info'),
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            image_topic_arg,
            camera_node,
        ]
    )
