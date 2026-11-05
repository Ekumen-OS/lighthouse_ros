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

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"

#include "lighthouse_deck_semantic_component/lighthouse_deck_semantic_component.hpp"

// implementing and friending so we can access member variables
class TestableLighthouseDeckSemanticComponent
    : public lighthouse_deck_semantic_component::
          LighthouseDeckSemanticComponent {
  FRIEND_TEST(LighthouseDeckSemanticComponentTest, validate_all);

public:
  explicit TestableLighthouseDeckSemanticComponent(const std::string &name)
      : LighthouseDeckSemanticComponent(name) {}
};

class LighthouseDeckSemanticComponentTest : public ::testing::Test {
public:
  void SetUp() {
    full_interface_names_ = {
        // Sensor 0
        "test_lighthouse_sensor_0_base_00/azimuth",
        "test_lighthouse_sensor_0_base_00/elevation",
        "test_lighthouse_sensor_0_base_01/azimuth",
        "test_lighthouse_sensor_0_base_01/elevation",
        "test_lighthouse_sensor_0_base_02/azimuth",
        "test_lighthouse_sensor_0_base_02/elevation",
        "test_lighthouse_sensor_0_base_03/azimuth",
        "test_lighthouse_sensor_0_base_03/elevation",
        // Sensor 1
        "test_lighthouse_sensor_1_base_00/azimuth",
        "test_lighthouse_sensor_1_base_00/elevation",
        "test_lighthouse_sensor_1_base_01/azimuth",
        "test_lighthouse_sensor_1_base_01/elevation",
        "test_lighthouse_sensor_1_base_02/azimuth",
        "test_lighthouse_sensor_1_base_02/elevation",
        "test_lighthouse_sensor_1_base_03/azimuth",
        "test_lighthouse_sensor_1_base_03/elevation",
        // Sensor 2
        "test_lighthouse_sensor_2_base_00/azimuth",
        "test_lighthouse_sensor_2_base_00/elevation",
        "test_lighthouse_sensor_2_base_01/azimuth",
        "test_lighthouse_sensor_2_base_01/elevation",
        "test_lighthouse_sensor_2_base_02/azimuth",
        "test_lighthouse_sensor_2_base_02/elevation",
        "test_lighthouse_sensor_2_base_03/azimuth",
        "test_lighthouse_sensor_2_base_03/elevation",
        // Sensor 3
        "test_lighthouse_sensor_3_base_00/azimuth",
        "test_lighthouse_sensor_3_base_00/elevation",
        "test_lighthouse_sensor_3_base_01/azimuth",
        "test_lighthouse_sensor_3_base_01/elevation",
        "test_lighthouse_sensor_3_base_02/azimuth",
        "test_lighthouse_sensor_3_base_02/elevation",
        "test_lighthouse_sensor_3_base_03/azimuth",
        "test_lighthouse_sensor_3_base_03/elevation",
        // Timestamps
        "test_lighthouse_base_00/timestamp",
        "test_lighthouse_base_01/timestamp",
        "test_lighthouse_base_02/timestamp",
        "test_lighthouse_base_03/timestamp",
    };
  }

  void TearDown() { lighthouse_component_.reset(nullptr); }

protected:
  const std::string component_name_ = "test_lighthouse";
  std::unique_ptr<TestableLighthouseDeckSemanticComponent>
      lighthouse_component_;
  std::vector<std::string> full_interface_names_;

  std::vector<double> interface_values_ = {
      // Sensor 0
      1.0, 0.1, // base_00: azimuth, elevation
      2.0, 0.2, // base_01: azimuth, elevation
      3.0, 0.3, // base_02: azimuth, elevation
      4.0, 0.4, // base_03: azimuth, elevation
      // Sensor 1
      5.0, 0.5, // base_00: azimuth, elevation
      6.0, 0.6, // base_01: azimuth, elevation
      7.0, 0.7, // base_02: azimuth, elevation
      8.0, 0.8, // base_03: azimuth, elevation
      // Sensor 2
      9.0, 0.9,  // base_00: azimuth, elevation
      10.0, 1.0, // base_01: azimuth, elevation
      11.0, 1.1, // base_02: azimuth, elevation
      12.0, 1.2, // base_03: azimuth, elevation
      // Sensor 3
      13.0, 1.3, // base_00: azimuth, elevation
      14.0, 1.4, // base_01: azimuth, elevation
      15.0, 1.5, // base_02: azimuth, elevation
      16.0, 1.6, // base_03: azimuth, elevation
      // Timestamps
      100.0, // base_00
      200.0, // base_01
      300.0, // base_02
      400.0  // base_03
  };
};

TEST_F(LighthouseDeckSemanticComponentTest, validate_all) {
  lighthouse_component_ =
      std::make_unique<TestableLighthouseDeckSemanticComponent>(
          component_name_);

  ASSERT_EQ(lighthouse_component_->name_, component_name_);

  ASSERT_EQ(lighthouse_component_->interface_names_.size(),
            full_interface_names_.size());
  ASSERT_EQ(lighthouse_component_->state_interfaces_.size(), 0);

  ASSERT_TRUE(std::equal(lighthouse_component_->interface_names_.begin(),
                         lighthouse_component_->interface_names_.end(),
                         full_interface_names_.begin(),
                         full_interface_names_.end()));

  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < full_interface_names_.size(); ++i) {
    size_t slash_pos = full_interface_names_[i].find('/');
    std::string interface_name = full_interface_names_[i].substr(0, slash_pos);
    std::string interface_type = full_interface_names_[i].substr(slash_pos + 1);

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        interface_name, interface_type, &interface_values_[i]));
  }

  std::vector<hardware_interface::LoanedStateInterface> loaned_state_interfaces;

  for (auto &state_interface : state_interfaces) {
    loaned_state_interfaces.emplace_back(state_interface);
  }

  lighthouse_component_->assign_loaned_state_interfaces(
      loaned_state_interfaces);

  ASSERT_EQ(lighthouse_component_->state_interfaces_.size(),
            full_interface_names_.size());

  lighthouse_component_->update_from_interfaces();
  auto base_station_data = lighthouse_component_->get_base_station_data();
  size_t value_idx = 0;
  for (size_t sensor = 0; sensor < 4; ++sensor) {
    for (size_t base_station = 0; base_station < 4; ++base_station) {
      ASSERT_DOUBLE_EQ(base_station_data[base_station].sensors[sensor].azimuth,
                       interface_values_[value_idx++]);
      ASSERT_DOUBLE_EQ(
          base_station_data[base_station].sensors[sensor].elevation,
          interface_values_[value_idx++]);
    }
  }

  for (size_t base_station = 0; base_station < 4; ++base_station) {
    ASSERT_DOUBLE_EQ(base_station_data[base_station].timestamp,
                     interface_values_[value_idx++]);
  }

  lighthouse_component_->release_interfaces();

  ASSERT_EQ(lighthouse_component_->state_interfaces_.size(), 0u);
}
