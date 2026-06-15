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
project(lighthouse_deck_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lighthouse_geometry_utils REQUIRED)
find_package(Sophus REQUIRED)

# Create the utils library
add_library(${PROJECT_NAME} SHARED
  src/serial_port.cpp
  src/utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${ASIO_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link pthread for threading support
target_link_libraries(${PROJECT_NAME} PUBLIC
  pthread
  Sophus::Sophus
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  lighthouse_geometry_utils
)

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit test for utils
  ament_add_gtest(test_utils test/test_utils.cpp)
  target_link_libraries(test_utils
    ${PROJECT_NAME}
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  asio_cmake_module
  ASIO
  rclcpp
  lighthouse_geometry_utils
  Sophus
)

ament_package()
