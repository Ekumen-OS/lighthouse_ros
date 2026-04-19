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
project(lighthouse_station_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lighthouse_deck_msgs REQUIRED)
find_package(lighthouse_geometry_utils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Fetch FTXUI
include(FetchContent)
fetchcontent_declare(
  ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
  GIT_TAG        v6.1.9
  GIT_SHALLOW    TRUE
)
fetchcontent_makeavailable(ftxui)

find_package(Sophus REQUIRED)

# Shared rendering library
add_library(mapper_ui_lib
  src/mapper_screen_renderer.cpp
)

target_include_directories(mapper_ui_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mapper_ui_lib PUBLIC
  ftxui::screen
  ftxui::dom
  ftxui::component
  Sophus::Sophus
)

ament_target_dependencies(mapper_ui_lib PUBLIC
  lighthouse_geometry_utils
)

# mapper_ui executable (ROS 2 node)
add_executable(mapper_ui
  src/mapper_ui_main.cpp
  src/mapper_ui_node.cpp
)

target_link_libraries(mapper_ui
  mapper_ui_lib
  Sophus::Sophus
)

ament_target_dependencies(mapper_ui
  rclcpp
  geometry_msgs
  lighthouse_deck_msgs
  lighthouse_geometry_utils
  tf2_ros
  visualization_msgs
)

install(TARGETS mapper_ui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
