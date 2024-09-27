from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="a",
        description="Serial port where the deck is connected",
    )

    return LaunchDescription([
        Node(
            package='lighthouse_ros',
            namespace='',
            executable='lighthouse_ros',
            parameters=[
                {
                    "serial_port": serial_port
                }
            ]
        ),
        serial_port_arg
    ])
