#!/usr/bin/env python3

# Copyright 2025 Gerardo Puga
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
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from lighthouse_ros_msgs.msg import SensorMeasurementsStamped
from lighthouse_ros_msgs.msg import SensorAngles
from lighthouse_ros_msgs.msg import LighthouseStationMap
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import spatialmath as sm
from spatialmath.base import r2q
import math as m
import random


class LighthouseTestbench(Node):
    def __init__(self):
        """Create a LighthouseTestbench."""
        super().__init__("lighthouse_testbench")

        self.declare_parameter("sensor_height", 0.0)

        self.__sensor_height = (
            self.get_parameter("sensor_height").get_parameter_value().double_value
        )

        self.__station_poses = {}
        self.__map_frame = "map"
        self.__odom_frame = "odom"
        self.__base_link_frame = "base_link"

        self.__timer_period_robot = 0.05
        self.__timer_period_sensor = 0.1

        self.__current_robot_pose = sm.SE2(2, 2, 0)
        self.__latest_twist_message = Twist()
        self.__latest_twist_message_timestamp = self.get_clock().now()

        self.__odom_offset = sm.SE2(2.5, 1.0, 0.45)

        self.__lighthouse_sensor_pub = self.create_publisher(
            SensorMeasurementsStamped,
            "lighthouse",
            10,
        )

        self.__robot_pose_pub = self.create_publisher(
            PoseStamped,
            "lighthouse_deck_groundtruth",
            10,
        )

        self.__odom_transform_broadcaster = TransformBroadcaster(self)

        self.__current_pose_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            1,
        )

        self.__lighthouse_map_sub = self.create_subscription(
            LighthouseStationMap,
            "lighthouse_map",
            self.lighthouse_map_callback,
            1,
        )

        self.__robot_pose_timer = self.create_timer(
            self.__timer_period_robot,
            self.timer_callback_robot_pose,
        )

        self.__lighthouse_sensor_timer = self.create_timer(
            self.__timer_period_sensor,
            self.timer_callback_sensor,
        )

    def cmd_vel_callback(self, msg):
        """Store the current pose of the robot."""
        self.__latest_twist_message = msg
        self.__latest_twist_message_timestamp = self.get_clock().now()

    def lighthouse_map_callback(self, msg):
        """Store the current pose of the robot."""
        for station in msg.station_poses:
            tr = sm.SE3(
                station.pose.position.x,
                station.pose.position.y,
                station.pose.position.z,
            )
            r = sm.UnitQuaternion(
                [
                    station.pose.orientation.w,
                    station.pose.orientation.x,
                    station.pose.orientation.y,
                    station.pose.orientation.z,
                ]
            ).SE3()
            station_pose = tr * r
            self.__station_poses[station.base_station_id] = station_pose
        self.__map_frame = msg.header.frame_id

    def timer_callback_sensor(self):
        """Publish a message for each base station."""
        x, y = self.__current_robot_pose.t
        theta = self.__current_robot_pose.theta()
        robot_pose_3D = sm.SE3.Trans(x, y, 0) * sm.SE3.Rz(theta)
        stations_tuples = self.__station_poses.items()
        for station_id, station_pose in random.sample(
            stations_tuples, len(stations_tuples)
        ):
            sensor_measurements = SensorMeasurementsStamped()
            sensor_measurements.header.stamp = self.get_clock().now().to_msg()
            sensor_measurements.header.frame_id = self.__map_frame
            sensor_measurements.sweep_data.base_station_id = station_id
            sensor_measurements.sweep_data.sensor_angles = self.calculate_sensor_angles(
                robot_pose_3D,
                station_pose,
            )
            self.__lighthouse_sensor_pub.publish(sensor_measurements)

    def calculate_sensor_angles(self, robot_pose, station_pose):
        """Calculate the angles of the sensors."""
        x_offset = 0.0296 / 2.0
        y_offset = 0.0150 / 2.0
        offsets = [
            sm.SE3(-x_offset, +y_offset, self.__sensor_height),
            sm.SE3(-x_offset, -y_offset, self.__sensor_height),
            sm.SE3(+x_offset, +y_offset, self.__sensor_height),
            sm.SE3(+x_offset, -y_offset, self.__sensor_height),
        ]
        sensor_angles = [
            self.calculate_singular_sensor_angle(robot_pose * offset, station_pose)
            for offset in offsets
        ]
        return sensor_angles

    def calculate_singular_sensor_angle(self, sensor_pose, station_pose):
        """Calculate the angle of a single sensor."""
        # calculate the sensor pose in the station frame
        robot_pose_in_sensor_frame = station_pose.inv() * sensor_pose
        # calculate the elevation and azimuth angles
        dx, dy, dz = robot_pose_in_sensor_frame.t
        azimuth = m.atan2(dy, dx)
        elevation = m.atan2(dz, m.sqrt(dx**2 + dy**2))
        return SensorAngles(
            elevation=elevation,
            azimuth=azimuth,
        )

    def timer_callback_robot_pose(self):
        """Update the current pose of the robot."""
        self.update_robot_pose()
        self.publish_robot_odom_tf()
        self.publish_robot_groundtruth()

    def update_robot_pose(self):
        """Update the current pose of the robot."""
        current_time = self.get_clock().now()
        if current_time - self.__latest_twist_message_timestamp > Duration(seconds=0.5):
            self.__latest_twist_message = Twist()
            self.__latest_twist_message_timestamp = self.get_clock().now()
        delta_pose = sm.SE2(
            self.__latest_twist_message.linear.x * self.__timer_period_robot,
            self.__latest_twist_message.linear.y * self.__timer_period_robot,
            self.__latest_twist_message.angular.z * self.__timer_period_robot,
        )
        self.__current_robot_pose = self.__current_robot_pose * delta_pose

    def publish_robot_odom_tf(self):
        """Publish the current pose of the robot as a TF."""
        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now() + Duration(seconds=0.05)
        ).to_msg()

        offset_pose = self.__odom_offset * self.__current_robot_pose

        transform.header.frame_id = self.__odom_frame
        transform.child_frame_id = self.__base_link_frame
        transform.transform.translation.x = offset_pose.x
        transform.transform.translation.y = offset_pose.y
        transform.transform.translation.z = 0.0
        quaternion = r2q(sm.SO3.RPY([0, 0, offset_pose.theta()]).R)
        transform.transform.rotation.w = quaternion[0]
        transform.transform.rotation.x = quaternion[1]
        transform.transform.rotation.y = quaternion[2]
        transform.transform.rotation.z = quaternion[3]
        self.__odom_transform_broadcaster.sendTransform(transform)

    def publish_robot_groundtruth(self):
        """Publish the current pose of the robot."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.__map_frame
        pose.pose.position.x = self.__current_robot_pose.x
        pose.pose.position.y = self.__current_robot_pose.y
        quaternion = r2q(sm.SO3.RPY([0, 0, self.__current_robot_pose.theta()]).R)
        pose.pose.orientation.w = quaternion[0]
        pose.pose.orientation.x = quaternion[1]
        pose.pose.orientation.y = quaternion[2]
        pose.pose.orientation.z = quaternion[3]
        self.__robot_pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = LighthouseTestbench()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
