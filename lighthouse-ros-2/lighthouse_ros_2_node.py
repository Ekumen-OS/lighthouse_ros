from core import LighthouseCore

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LighthouseNode(Node):

    def __init__(self):
        super().__init__('lighthouse_ros_2')
        # TODO: Modify with custom message
        publisher = self.create_publisher(String, 'topic', 10)
        # TODO Get serial devidce parameter to be able to instantiate serial handle and pass it to core
        # if len(sys.argv) < 2:
        #     print("Usage: {} <input.bin or /dev/tty...>".format(sys.argv[0]))
        #     exit(1)

        # serial_handler = SerialHandler(sys.argv[1])
        # TODO: Instantiate Core and pass the publisher
        self._core = LighthouseCore()


def main(args=None):
    rclpy.init(args=args)

    lighthouse_node = LighthouseNode()

    rclpy.spin(lighthouse_node)

    lighthouse_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
