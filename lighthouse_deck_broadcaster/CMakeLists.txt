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
project(lighthouse_deck_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lighthouse_deck_semantic_component REQUIRED)
find_package(lighthouse_deck_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

# Create the broadcaster library
add_library(${PROJECT_NAME} SHARED
  src/lighthouse_deck_broadcaster.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  lighthouse_deck_semantic_component
  lighthouse_deck_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

# Export the plugin
pluginlib_export_plugin_description_file(
  controller_interface lighthouse_deck_broadcaster.xml)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES lighthouse_deck_broadcaster.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_lighthouse_deck_broadcaster
    test/test_load_lighthouse_deck_broadcaster.cpp)
  target_include_directories(test_load_lighthouse_deck_broadcaster
    PRIVATE include)
  target_link_libraries(test_load_lighthouse_deck_broadcaster
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_load_lighthouse_deck_broadcaster
    controller_interface
    controller_manager
    hardware_interface
    lifecycle_msgs
    lighthouse_deck_semantic_component
    lighthouse_deck_msgs
    rclcpp
    rclcpp_lifecycle
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  hardware_interface
  lighthouse_deck_semantic_component
  lighthouse_deck_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

ament_package()
