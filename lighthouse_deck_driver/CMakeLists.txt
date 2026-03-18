# Copyright 2026 Ekumen, Inc.
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

cmake_minimum_required(VERSION 3.8)
project(lighthouse_deck_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lighthouse_deck_msgs REQUIRED)
find_package(lighthouse_protocol_decoder REQUIRED)
find_package(lighthouse_deck_utils REQUIRED)

# Create the lighthouse deck driver component library
add_library(lighthouse_deck_driver_component SHARED
  src/lighthouse_deck_driver_node.cpp
)

target_include_directories(lighthouse_deck_driver_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(lighthouse_deck_driver_component
  rclcpp
  rclcpp_components
  std_msgs
  lighthouse_deck_msgs
  lighthouse_protocol_decoder
  lighthouse_deck_utils
)

# Register the component
rclcpp_components_register_node(lighthouse_deck_driver_component
  PLUGIN "lighthouse_deck_driver::LighthouseDeckDriverNode"
  EXECUTABLE lighthouse_deck_driver
)

# Install the component library
install(TARGETS lighthouse_deck_driver_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  lighthouse_deck_msgs
  lighthouse_protocol_decoder
  lighthouse_deck_utils
)

ament_package()
