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

cmake_minimum_required(VERSION 3.16)
project(lighthouse_amcl)

find_package(ament_cmake REQUIRED)
find_package(beluga REQUIRED)
find_package(beluga_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lighthouse_ros_msgs REQUIRED)
find_package(lighthouse_sensor_model REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(lighthouse_amcl_component SHARED)
target_sources(lighthouse_amcl_component
  PRIVATE
    src/lighthouse_amcl.cpp
)
target_compile_features(lighthouse_amcl_component PUBLIC cxx_std_17)
target_link_libraries(lighthouse_amcl_component
  PUBLIC
    beluga::beluga
    beluga_ros::beluga_ros
    lighthouse_sensor_model::lighthouse_sensor_model
)
ament_target_dependencies(
  lighthouse_amcl_component
  PUBLIC
    geometry_msgs
    lighthouse_ros_msgs
    lighthouse_sensor_model
    message_filters
    rclcpp
    rclcpp_components
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)
rclcpp_components_register_node(
  lighthouse_amcl_component
  PLUGIN
  "lighthouse_amcl::LighthouseMCLNode"
  EXECUTABLE lighthouse_amcl_node)

install(
  TARGETS lighthouse_amcl_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS lighthouse_amcl_component DESTINATION lib/${PROJECT_NAME})

ament_package()
