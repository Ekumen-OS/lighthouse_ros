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
project(lighthouse_protocol_decoder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Fetch cppcrc library
include(FetchContent)
FetchContent_Declare(
  cppcrc
  GIT_REPOSITORY https://github.com/DarrenLevine/cppcrc.git
  GIT_TAG 6360eee1c8966d32b2552ce156b135ec6a3235f1
)
FetchContent_MakeAvailable(cppcrc)

# Library
add_library(${PROJECT_NAME} SHARED
  src/crc32.cpp
  src/data_frame_contents.cpp
  src/data_frame_decoder.cpp
  src/ootx_frame_decoder.cpp
  src/sync_frame_decoder.cpp
  src/sweep_processor.cpp
  src/measurement_processor.cpp
  src/lighthouse_protocol_decoder.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${cppcrc_SOURCE_DIR}
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
ament_export_dependencies()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test helpers library
  add_library(test_helpers
    test/test_helpers.cpp
  )

  target_include_directories(test_helpers
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_helpers
    ${PROJECT_NAME}
  )

  target_compile_features(test_helpers PUBLIC cxx_std_17)

  ament_add_gtest(test_ootx_frame_decoder
    test/test_ootx_frame_decoder.cpp
  )

  target_link_libraries(test_ootx_frame_decoder
    ${PROJECT_NAME}
    test_helpers
  )

  ament_add_gtest(test_sync_frame_decoder
    test/test_sync_frame_decoder.cpp
  )

  target_link_libraries(test_sync_frame_decoder
    ${PROJECT_NAME}
    test_helpers
  )

  ament_add_gtest(test_data_frame_decoder
    test/test_data_frame_decoder.cpp
  )

  target_link_libraries(test_data_frame_decoder
    ${PROJECT_NAME}
    test_helpers
  )

  ament_add_gtest(test_crc32
    test/test_crc32.cpp
  )

  target_link_libraries(test_crc32
    ${PROJECT_NAME}
    test_helpers
  )

  ament_add_gtest(test_sweep_processor
    test/test_sweep_processor.cpp
  )

  target_link_libraries(test_sweep_processor
    ${PROJECT_NAME}
    test_helpers
  )

  ament_add_gtest(test_measurement_processor
    test/test_measurement_processor.cpp
  )

  target_link_libraries(test_measurement_processor
    ${PROJECT_NAME}
    test_helpers
  )

  ament_add_gtest(test_lighthouse_protocol_decoder
    test/test_lighthouse_protocol_decoder.cpp
  )

  target_link_libraries(test_lighthouse_protocol_decoder
    ${PROJECT_NAME}
    test_helpers
  )
endif()

ament_package()
