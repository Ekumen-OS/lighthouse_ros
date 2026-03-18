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

from lighthouse_ros_msgs.msg import LighthouseStationMap
from lighthouse_ros_msgs.msg import LighthouseStationPose

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.node import Node

import spatialmath as sm
from spatialmath.base import r2q
import yaml


class LighthouseMapServerNode(Node):
    def __init__(self):
        """Create a LighthouseMapServerNode."""
        super().__init__("lighthouse_map_publisher")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("station_poses_path", "")

        self.__map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )

        self.__station_poses_path = (
            self.get_parameter("station_poses_path").get_parameter_value().string_value
        )

        self.__station_poses = self.load_station_poses_from_file(
            self.__station_poses_path
        )

        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.__station_map_pub = self.create_publisher(
            LighthouseStationMap,
            "lighthouse_map",
            qos_profile,
        )

        self.__visualization_markers_pub = self.create_publisher(
            MarkerArray,
            "lighthouse_markers",
            qos_profile,
        )

        self.timer_callback()

    def timer_callback(self):
        """Publish the map and markers."""
        self.publish_station_map()
        self.publish_station_markers()

    def publish_station_map(self):
        """Publish the map of the base stations."""
        station_map = LighthouseStationMap()
        station_map.header.stamp = self.get_clock().now().to_msg()
        station_map.header.frame_id = self.__map_frame
        for station_id, station_pose in self.__station_poses.items():
            new_entry = LighthouseStationPose()
            new_entry.base_station_id = station_id
            new_entry.pose.position.x = station_pose.x
            new_entry.pose.position.y = station_pose.y
            new_entry.pose.position.z = station_pose.z
            quaternion = r2q(station_pose.R)
            new_entry.pose.orientation.w = quaternion[0]
            new_entry.pose.orientation.x = quaternion[1]
            new_entry.pose.orientation.y = quaternion[2]
            new_entry.pose.orientation.z = quaternion[3]
            station_map.station_poses.append(new_entry)
        self.__station_map_pub.publish(station_map)

    def build_station_marker(self, station_id, station_pose):
        """Build the marker for a base station."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.__map_frame
        marker.ns = "lighthouse"
        marker.id = 100 + station_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = station_pose.x
        marker.pose.position.y = station_pose.y
        marker.pose.position.z = station_pose.z
        quaternion = r2q(station_pose.R)
        marker.pose.orientation.w = quaternion[0]
        marker.pose.orientation.x = quaternion[1]
        marker.pose.orientation.y = quaternion[2]
        marker.pose.orientation.z = quaternion[3]
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.25
        marker.color.a = 0.75
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker

    def publish_station_markers(self):
        """Publish the markers for the base stations."""
        markers = MarkerArray()
        for station_id, station_pose in self.__station_poses.items():
            markers.markers.append(self.build_station_marker(station_id, station_pose))
        self.__visualization_markers_pub.publish(markers)

    def load_station_poses_from_file(self, filename):
        """Load the goals from a file."""
        station_poses = {}
        try:
            with open(filename, "r") as file:
                station_in_file = yaml.safe_load(file)
            for item in station_in_file:
                translation = sm.SE3(item["x"], item["y"], item["z"])
                rotation = sm.SE3.RPY(item["roll"], item["pitch"], item["yaw"])
                id = int(item["id"])
                station_poses[id] = translation * rotation
        except FileNotFoundError:
            self.get_logger().error("File not found: %s" % filename)
        except yaml.YAMLError as e:
            self.get_logger().error("Error parsing YAML file: %s" % e)
        except Exception as e:
            self.get_logger().error("Error loading file: %s" % e)
        return station_poses


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = LighthouseMapServerNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
