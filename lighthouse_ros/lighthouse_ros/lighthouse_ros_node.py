from lighthouse_ros.core import LighthouseCore
from lighthouse_ros.serial_handler import SerialHandler
from lighthouse_ros_msgs.msg import SensorMeasurements

import rclpy
import rclpy.logging
from rclpy.node import Node
from serial import SerialException

class LighthouseNode(Node):

    def __init__(self):
        super().__init__('lighthouse_ros')
        logger = self.get_logger()

        # Message publishers
        publisher = self.create_publisher(SensorMeasurements, 'sensor_measurements', 10)

        # Serial port and handler
        self.declare_parameter('serial_port', '')
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        if not serial_port:
            logger.fatal("Serial port argument is required")
            raise RuntimeError
        try:
            serial_handler = SerialHandler(serial_port)
        except (SerialException, FileNotFoundError) as e:
            logger.fatal("Could not open serial port")
            raise e

        self._core = LighthouseCore(serial_handler, publisher)


def main(args=None):
    rclpy.init(args=args)

    try:
        lighthouse_node = LighthouseNode()

        rclpy.spin(lighthouse_node)

        lighthouse_node.destroy_node()
        rclpy.shutdown()
    except Exception:
        return


if __name__ == '__main__':
    main()
