# Copyright 2025 Ekumen, Inc.
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

import rclpy
from rclpy.node import Node
import numpy as np
import math
from lighthouse_ros_msgs.msg import SensorMeasurementsStamped
from .base_geometry_simple_estimation import BaseGeometrySimpleEstimator


class BaseGeometryEstimationNode(Node):
    """
    A node to estimate Lighthouse base stations geometry.

    This node subscribes to the '/lighthouse' topic, collects sensor measurements
    for a fixed time duration, and then uses the collected data to estimate the
    position and orientation of each visible base station relative to the
    sensor deck.
    """

    def __init__(self):
        """Initialize the node, subscription, and data collection timer."""
        super().__init__('base_geometry_estimation_node')
        self.get_logger().info("Estimation: starting node")

        self._origin_messages = (
            []
        )  # Messages collected at the origin of the new frame of reference
        self._is_registering = True

        self._subscription = self.create_subscription(
            SensorMeasurementsStamped, '/lighthouse', self.listener_callback, 10
        )

        self._data_collection_duration = 5  # In seconds
        self._timer = self.create_timer(
            self._data_collection_duration, self.stop_registering
        )

    def listener_callback(self, msg):
        """
        Process the SensorMeasurementsStamped measurements.

        If the node is in the registering state, it registers the data from the
        SensorMeasurementsStamped message.

        Args:
            msg (SensorMeasurementsStamped): The incoming message.
        """
        if self._is_registering:
            base_station_id = msg.sweep_data.base_station_id
            sensor_angles = msg.sweep_data.sensor_angles
            self._origin_messages.append(
                np.array(
                    [
                        base_station_id,
                        np.deg2rad(sensor_angles[0].elevation),
                        np.deg2rad(sensor_angles[0].azimuth),
                        np.deg2rad(sensor_angles[1].elevation),
                        np.deg2rad(sensor_angles[1].azimuth),
                        np.deg2rad(sensor_angles[2].elevation),
                        np.deg2rad(sensor_angles[2].azimuth),
                        np.deg2rad(sensor_angles[3].elevation),
                        np.deg2rad(sensor_angles[3].azimuth),
                    ]
                )
            )

    def stop_registering(self):
        """Stop the message registration process and trigger estimation."""
        self._is_registering = False
        self.get_logger().info(
            f"Stopped registering messages after {self._data_collection_duration} seconds."
        )
        self._timer.cancel()

        if self._origin_messages:
            self.get_logger().info(f"Received {len(self._origin_messages)} messages.\n")
            self.estimation()
        else:
            self.get_logger().info(
                f"No messages received in the last {self._data_collection_duration} seconds."
            )

        self.destroy_subscription(self._subscription)
        # TODO(Nico): this is not making the system to exit
        self.destroy_node()

    def get_averaged_data_per_base(self, raw_data: np.array):
        """
        Process sensor data to get an average of the angles per base station.

        Groups the collected data by base station ID and computes the
        average of all sensor angle measurements for each station.

        Args:
            raw_data (np.array): An array where each row contains a
                base station ID followed by 8 sensor angle values.

        Returns:
            dict: A dictionary mapping each base station ID to a numpy
                  array of its averaged sensor angles.
        """
        # Group data by base_station_id
        unique_base_ids = np.unique(raw_data[:, 0])

        # Create a dictionary to store the subarrays for each id
        data_per_base_dict = {}

        for id in unique_base_ids:
            # Filter rows where the id matches
            data_per_base_dict[id] = raw_data[raw_data[:, 0] == id]
            # Remove the id column
            data_per_base_dict[id] = data_per_base_dict[id][:, 1:]

        # Average data for all bases per column and convert to radians
        for id, data_per_base in data_per_base_dict.items():
            data_per_base_dict[id] = np.average(data_per_base, axis=0)

        return data_per_base_dict

    def estimation(self):
        """
        Perform the base station geometry estimation.

        This method takes the collected sensor data, runs the
        `BaseGeometrySimpleEstimator` for each detected base station, and
        prints the resulting position and rotation to console. The
        positions are normalized by the largest absolute coordinate value
        found across all estimations.
        """
        if not self._origin_messages:
            self.get_logger().info("No data to run estimation.\n")
            return

        averaged_origin_data_per_base = self.get_averaged_data_per_base(
            np.array(self._origin_messages)
        )
        self.get_logger().info(
            f"Running estimation for {len(averaged_origin_data_per_base)} base stations.\n"
        )

        estimator = BaseGeometrySimpleEstimator()
        base_station_data = {}
        max_abs_value = -math.inf
        for base_station_id, data_per_base in averaged_origin_data_per_base.items():
            try:
                retval, rotation, position = estimator.estimate_geometry(data_per_base)
                if retval:
                    base_station_data[base_station_id] = (position, rotation)
                    max_abs_value = max(
                        max_abs_value,
                        max(np.abs(np.min(position)), np.abs(np.max(position))),
                    )

            except Exception as e:
                self.get_logger().info(
                    f"Could not estimate geometry for base station {base_station_id}: {e}"
                )

        for base_station_id, (position, rotation) in base_station_data.items():
            if max_abs_value != 0:
                position = position / max_abs_value
            self.get_logger().info(f"Base station {int(base_station_id)}:")
            self.get_logger().info(f"  - Position: {position.flatten()}")
            self.get_logger().info(f"  - Rotation:\n{rotation}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BaseGeometryEstimationNode()
        rclpy.spin(node)
    except (ValueError, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
