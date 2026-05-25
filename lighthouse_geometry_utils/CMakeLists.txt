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
project(lighthouse_geometry_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

# Library
add_library(${PROJECT_NAME} SHARED
  src/ceres_helpers.cpp
  src/deck_pose_optimization.cpp
  src/station_pose_pnp_solver.cpp
  src/station_geometry_initialization.cpp
  src/station_geometry_optimization.cpp
  src/station_angles_utils.cpp
  src/utils.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  Ceres::ceres
  ${OpenCV_LIBS}
  Sophus::Sophus
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Ceres OpenCV Sophus)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Test helpers library
  add_library(test_helpers
    test/test_helpers.cpp
  )

  target_include_directories(test_helpers
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${OpenCV_INCLUDE_DIRS}
  )

  target_link_libraries(test_helpers
    ${PROJECT_NAME}
  )

  target_compile_features(test_helpers PUBLIC cxx_std_17)

  ament_add_gtest(test_station_pose_pnp_solver
    test/test_station_pose_pnp_solver.cpp
  )

  if(TARGET test_station_pose_pnp_solver)
    target_link_libraries(test_station_pose_pnp_solver
      ${PROJECT_NAME}
      test_helpers
    )
  endif()

  ament_add_gtest(test_station_geometry_optimization
    test/test_station_geometry_optimization.cpp
    TIMEOUT 120
  )

  if(TARGET test_station_geometry_optimization)
    target_link_libraries(test_station_geometry_optimization
      ${PROJECT_NAME}
      test_helpers
    )
  endif()

  ament_add_gtest(test_deck_pose_optimization
    test/test_deck_pose_optimization.cpp
  )

  if(TARGET test_deck_pose_optimization)
    target_link_libraries(test_deck_pose_optimization
      ${PROJECT_NAME}
      test_helpers
    )
  endif()

  ament_add_gtest(test_station_geometry_initialization
    test/test_station_geometry_initialization.cpp
  )

  if(TARGET test_station_geometry_initialization)
    target_link_libraries(test_station_geometry_initialization
      ${PROJECT_NAME}
      test_helpers
    )
  endif()

  ament_add_gtest(test_real_world_measurements
    test/test_real_world_measurements.cpp
  )

  if(TARGET test_real_world_measurements)
    target_link_libraries(test_real_world_measurements
      ${PROJECT_NAME}
      test_helpers
    )
  endif()

  ament_add_gtest(test_test_helpers
    test/test_test_helpers.cpp
  )

  if(TARGET test_test_helpers)
    target_link_libraries(test_test_helpers
      ${PROJECT_NAME}
      test_helpers
    )
  endif()

  ament_add_gtest(test_real_data_collinearity
    test/test_real_data_collinearity.cpp
  )

  if(TARGET test_real_data_collinearity)
    target_link_libraries(test_real_data_collinearity
      ${PROJECT_NAME}
      test_helpers
    )
  endif()
endif()

ament_package()
