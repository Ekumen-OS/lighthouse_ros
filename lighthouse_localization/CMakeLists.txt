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
project(lighthouse_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lighthouse_deck_msgs REQUIRED)
find_package(lighthouse_deck_utils REQUIRED)
find_package(lighthouse_geometry_utils REQUIRED)
find_package(lighthouse_station_mapper_msgs REQUIRED)
find_package(Sophus REQUIRED)

# lighthouse_localization executable (ROS 2 node)
add_executable(lighthouse_localization
  src/lighthouse_localization_main.cpp
  src/lighthouse_localization_node.cpp
)

target_include_directories(lighthouse_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(lighthouse_localization
  Sophus::Sophus
)

ament_target_dependencies(lighthouse_localization
  rclcpp
  geometry_msgs
  lighthouse_deck_msgs
  lighthouse_deck_utils
  lighthouse_geometry_utils
  lighthouse_station_mapper_msgs
)

install(TARGETS lighthouse_localization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
