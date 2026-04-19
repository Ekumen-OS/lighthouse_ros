// Copyright 2026 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestLighthouseDeckHardware : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_lighthouse_deck_ =
      R"(
  <ros2_control name="LighthouseDeckHardware" type="sensor">
    <hardware>
      <plugin>lighthouse_deck_hardware/LighthouseDeckHardware</plugin>
      <param name="device">/dev/ttyUSB99</param>
      <param name="baudrate">19200</param>
    </hardware>
    <sensor name="lighthouse_sensor_0_base_00">
      <state_interface name="azimuth"/>
      <state_interface name="elevation"/>
    </sensor>
    <sensor name="lighthouse_sensor_0_base_01">
      <state_interface name="azimuth"/>
      <state_interface name="elevation"/>
    </sensor>
    <sensor name="lighthouse_base_00">
      <state_interface name="timestamp"/>
    </sensor>
    <sensor name="lighthouse_base_01">
      <state_interface name="timestamp"/>
    </sensor>
  </ros2_control>
)";
  }

  std::string hardware_lighthouse_deck_;
};

TEST_F(TestLighthouseDeckHardware, load_lighthouse_deck_hardware) {
  auto urdf = ros2_control_test_assets::urdf_head + hardware_lighthouse_deck_ +
    ros2_control_test_assets::urdf_tail;
  // sadly ASSERT_NO_THROW does not show information about what was thrown
  try {
    hardware_interface::ResourceManager uut(urdf);
  } catch (const std::exception & e) {
    FAIL() << "Exception thrown while loading LighthouseDeckHardware: "
           << e.what();
  } catch (...) {
    FAIL() << "Unknown exception thrown while loading LighthouseDeckHardware";
  }
}
