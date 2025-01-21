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

from lighthouse_ros.lighthouse_protocol import LighthouseProtocol
from lighthouse_ros.lighthouse_protocol_decoder import (
    LighthouseProtocolDecoder,
)
from lighthouse_ros.types import SweepBlockBearings

from lighthouse_ros_msgs.msg import SensorMeasurementsStamped
from lighthouse_ros_msgs.msg import SensorAngles

from serial import Serial
from serial.threaded import ReaderThread

import rclpy
from rclpy.node import Node

import time
from smbus2 import SMBus

class LighthouseNode(Node):
    VALID_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400]

    def __init__(self):
        """Initialize the node."""
        super().__init__("lighthouse_ros")
        self.__logger = self.get_logger()

        self.__logger.info("Starting Lighthouse ROS node")

        # Serial port and handler
        self.declare_parameter("device", "")
        self.declare_parameter("baudrate", 230400)

        self.__device = self.get_parameter("device").get_parameter_value().string_value
        self.__baudrate = (
            self.get_parameter("baudrate").get_parameter_value().integer_value
        )

        if not self.__device:
            self.__logger.fatal("Serial port argument is required")
            raise RuntimeError

        if self.__baudrate not in self.VALID_BAUDRATES:
            self.__logger.fatal(
                "Invalid baudrate, valid options are: {}".format(self.VALID_BAUDRATES)
            )
            raise RuntimeError

        self.__logger.info("Device: {}".format(self.__device))
        self.__logger.info("Baudrate: {}".format(self.__baudrate))

        self.__bearings_publisher = self.create_publisher(
            SensorMeasurementsStamped, "lighthouse", 10
        )

        self.__src_device = Serial(
            self.__device,
            self.__baudrate,
        )

        self._light_house_handler = LighthouseProtocolDecoder(
            logger=self.__logger,
            app_bearing_callback=self.bearing_callback,
        )

        def build_factory(decoder_instance: LighthouseProtocolDecoder):
            def return_instance() -> LighthouseProtocol:
                return LighthouseProtocol(decoder_instance)

            return return_instance

        self.__reader_thread = ReaderThread(
            self.__src_device, build_factory(self._light_house_handler)
        )

        self.__reader_thread.start()
        self.__reader_thread.connect()

        # Write a 0 to get out of the bootloader mode and start receiving data via UART
        try:
            i2c_address = 0x2f
            bus = SMBus(1)
            time.sleep(1)
            var = bus.write_byte_data(i2c_address, 0, 0)
        except:
            self.__logger.info("Out of bootloader mode")

        self.__logger.info("Lighthouse ROS node started")

    def stop_thread(self):
        """Make sure we stop the background thread before we close the node."""
        self.__reader_thread.close()

    def bearing_callback(self, sensor_bearings: SweepBlockBearings):
        """Process the bearing measurements."""
        try:
            msg = SensorMeasurementsStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            # TODO we should assign a frame id based on the base_station_id and configuration
            msg.header.frame_id = ""
            msg.sweep_data.hardware_timestamp = sensor_bearings.hardware_timestamp
            msg.sweep_data.base_station_id = sensor_bearings.base_station_id
            for i in range(len(sensor_bearings.sensor_angles)):
                angles = SensorAngles()
                angles.azimuth = sensor_bearings.sensor_angles[i].azimuth
                angles.elevation = sensor_bearings.sensor_angles[i].elevation
                msg.sweep_data.sensor_angles[i] = angles
            self.__bearings_publisher.publish(msg)
        except Exception as e:
            self.__logger.error(f"Error processing bearing measurements: {e}")
            pass


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LighthouseNode()
        rclpy.spin(node)
        node.stop_thread()
        rclpy.shutdown()
    except Exception as e:
        print(f"Exception: {e}")
        return


if __name__ == "__main__":
    main()
