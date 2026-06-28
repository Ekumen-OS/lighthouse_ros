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

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lighthouse_deck_broadcaster/lighthouse_deck_broadcaster.hpp"
#include "lighthouse_deck_msgs/msg/lighthouse_deck_measurement.hpp"
#include "lighthouse_deck_semantic_component/lighthouse_deck_semantic_component.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using hardware_interface::LoanedStateInterface;
using testing::IsEmpty;
using testing::SizeIs;

namespace {
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;

} // namespace

// Extending Broadcaster to access protected members
class FriendLighthouseDeckBroadcaster
    : public lighthouse_deck_broadcaster::LighthouseDeckBroadcaster {
public:
  friend class LighthouseDeckBroadcasterTest;

  FRIEND_TEST(LighthouseDeckBroadcasterTest, ConfigureFailsWithoutDeviceName);
  FRIEND_TEST(LighthouseDeckBroadcasterTest, ConfigureSuccessWithDeviceName);
  FRIEND_TEST(LighthouseDeckBroadcasterTest, ActivateSuccess);
};

class LighthouseDeckBroadcasterTest : public ::testing::Test {
public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp() override {
    broadcaster_ = std::make_unique<FriendLighthouseDeckBroadcaster>();

    // Initialize test values
    for (size_t i = 0; i < sensor_values_.size(); ++i) {
      sensor_values_[i] = static_cast<double>(i) * 0.1;
    }

    initAndCreateInterfaces();
  }

  void TearDown() { broadcaster_.reset(nullptr); }

protected:
  std::string makeSensorInterfaceName(size_t sensor,
                                      size_t base_station) const {
    return device_name_ + "_sensor_" + std::to_string(sensor) + "_base_" +
           (base_station < 10 ? "0" : "") + std::to_string(base_station);
  }

  std::string makeBaseStationInterfaceName(size_t base_station) const {
    return device_name_ + "_base_" + (base_station < 10 ? "0" : "") +
           std::to_string(base_station);
  }

  void initAndCreateInterfaces() {
    const auto result = broadcaster_->init("test_lighthouse_deck_broadcaster");
    ASSERT_EQ(result, controller_interface::return_type::OK);

    loaned_state_interfaces_.clear();

    // create per sensor/base station interfaces
    size_t value_idx = 0;
    for (size_t sensor = 0; sensor < 4; ++sensor) {
      for (size_t base_station = 0; base_station < 4; ++base_station) {
        std::string interface_name =
            makeSensorInterfaceName(sensor, base_station);

        state_interfaces_.push_back(hardware_interface::StateInterface{
            interface_name, "azimuth", &sensor_values_[value_idx]});
        state_interfaces_.push_back(hardware_interface::StateInterface{
            interface_name, "elevation", &sensor_values_[value_idx + 1]});

        value_idx += 2;
      }
    }

    // Add timestamp interfaces
    for (size_t base_station = 0; base_station < 4; ++base_station) {
      std::string interface_name = makeBaseStationInterfaceName(base_station);
      state_interfaces_.emplace_back(interface_name, "timestamp",
                                     &sensor_values_[value_idx++]);
    }

    // Create LoanedStateInterface from StateInterface
    for (auto &state_itf : state_interfaces_) {
      loaned_state_interfaces_.emplace_back(state_itf);
    }

    broadcaster_->assign_interfaces({}, std::move(loaned_state_interfaces_));
  }

  void subscribe_and_get_messages(
      std::vector<lighthouse_deck_msgs::msg::LighthouseDeckMeasurement>
          &messages) {

    rclcpp::Node test_subscription_node("test_subscription_node");

    messages.clear();

    auto subs_callback =
        [&messages](const lighthouse_deck_msgs::msg::LighthouseDeckMeasurement::
                        SharedPtr msg) { messages.push_back(*msg); };

    auto subscription = test_subscription_node.create_subscription<
        lighthouse_deck_msgs::msg::LighthouseDeckMeasurement>(
        "lighthouse", rclcpp::SystemDefaultsQoS(), subs_callback);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(test_subscription_node.get_node_base_interface());

    // Call update to publish the test values. Since update doesn't guarantee
    // a published message, republish until received
    const auto max_retries = 5;
    for (int i = 0; i < max_retries && messages.empty(); ++i) {
      RCLCPP_WARN(test_subscription_node.get_logger(),
                  "Publishing lighthouse deck data, attempt %d/%d", i + 1,
                  max_retries);
      broadcaster_->update(rclcpp::Time(0),
                           rclcpp::Duration::from_seconds(0.01));

      const auto start = std::chrono::steady_clock::now();
      const auto until = start + std::chrono::milliseconds(100);
      while (std::chrono::steady_clock::now() < until && messages.empty()) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
  }

  const std::string device_name_ = "lighthouse";
  const std::string frame_id_ = "lighthouse_deck";

  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface>
      loaned_state_interfaces_;

  // Test values for 4 sensors x 4 base stations (azimuth, elevation) + 4
  // timestamps
  std::array<double, 36> sensor_values_;

  std::unique_ptr<FriendLighthouseDeckBroadcaster> broadcaster_;
};

void LighthouseDeckBroadcasterTest::SetUpTestCase() {
  rclcpp::init(0, nullptr);
}

void LighthouseDeckBroadcasterTest::TearDownTestCase() { rclcpp::shutdown(); }

TEST_F(LighthouseDeckBroadcasterTest, ConfigureFailsWithoutDeviceName) {
  // Do not set 'device_name' parameter
  broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});
  // Configure should fail
  ASSERT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

TEST_F(LighthouseDeckBroadcasterTest, ConfigureSuccessWithDeviceName) {
  // Set the 'device_name' and 'frame_id' parameters
  broadcaster_->get_node()->set_parameter({"device_name", device_name_});
  broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});
  // Configure should pass
  ASSERT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            NODE_SUCCESS);

  // Check interface configuration
  auto cmd_if_conf = broadcaster_->command_interface_configuration();
  ASSERT_THAT(cmd_if_conf.names, IsEmpty());
  EXPECT_EQ(cmd_if_conf.type,
            controller_interface::interface_configuration_type::NONE);

  auto state_if_conf = broadcaster_->state_interface_configuration();
  // 4 sensors x 4 base stations x 2 (azimuth + elevation) + 4 timestamps =
  ASSERT_THAT(state_if_conf.names, SizeIs(36lu));
  EXPECT_EQ(state_if_conf.type,
            controller_interface::interface_configuration_type::INDIVIDUAL);
}

TEST_F(LighthouseDeckBroadcasterTest, ActivateSuccess) {
  // Set required parameters
  broadcaster_->get_node()->set_parameter({"device_name", device_name_});
  broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  // Configure and activate should succeed
  ASSERT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            NODE_SUCCESS);
  // Activate should pass
  ASSERT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  // Deactivate should pass
  ASSERT_EQ(broadcaster_->on_deactivate(rclcpp_lifecycle::State()),
            NODE_SUCCESS);
}

TEST_F(LighthouseDeckBroadcasterTest, PublishSuccess) {
  // Set required parameters
  broadcaster_->get_node()->set_parameter({"device_name", device_name_});
  broadcaster_->get_node()->set_parameter({"frame_id", frame_id_});

  ASSERT_EQ(broadcaster_->on_configure(rclcpp_lifecycle::State()),
            NODE_SUCCESS);
  ASSERT_EQ(broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  std::vector<lighthouse_deck_msgs::msg::LighthouseDeckMeasurement> messages;
  subscribe_and_get_messages(messages);

  // We should receive 1 message with all base station data
  ASSERT_EQ(messages.size(), 1u);

  const auto &msg = messages[0];

  // Check header
  EXPECT_EQ(msg.header.frame_id, frame_id_);

  // Check that all vectors have the same length (4 base stations)
  ASSERT_EQ(msg.station_id.size(), 4u);
  ASSERT_EQ(msg.azimuth_0.size(), 4u);
  ASSERT_EQ(msg.azimuth_1.size(), 4u);
  ASSERT_EQ(msg.azimuth_2.size(), 4u);
  ASSERT_EQ(msg.azimuth_3.size(), 4u);
  ASSERT_EQ(msg.elevation_0.size(), 4u);
  ASSERT_EQ(msg.elevation_1.size(), 4u);
  ASSERT_EQ(msg.elevation_2.size(), 4u);
  ASSERT_EQ(msg.elevation_3.size(), 4u);

  // Verify each base station's data
  for (size_t base_station_id = 0; base_station_id < 4; ++base_station_id) {
    // Check station ID
    EXPECT_EQ(msg.station_id[base_station_id],
              static_cast<int32_t>(base_station_id));

    // Check sensor angles for all 4 sensors
    auto calc_index = [&](size_t sensor, size_t base) {
      return sensor * 8 +
             base * 2; // 4 sensors x 8 values each (azimuth + elevation)
    };
    size_t sensor_0_azimuth_idx = calc_index(0, base_station_id) + 0;
    size_t sensor_0_elevation_idx = calc_index(0, base_station_id) + 1;
    size_t sensor_1_azimuth_idx = calc_index(1, base_station_id) + 0;
    size_t sensor_1_elevation_idx = calc_index(1, base_station_id) + 1;
    size_t sensor_2_azimuth_idx = calc_index(2, base_station_id) + 0;
    size_t sensor_2_elevation_idx = calc_index(2, base_station_id) + 1;
    size_t sensor_3_azimuth_idx = calc_index(3, base_station_id) + 0;
    size_t sensor_3_elevation_idx = calc_index(3, base_station_id) + 1;

    ASSERT_EQ(msg.azimuth_0[base_station_id],
              sensor_values_[sensor_0_azimuth_idx])
        << "Mismatch for sensor 0, base station " << base_station_id
        << ", azimuth";

    ASSERT_EQ(msg.elevation_0[base_station_id],
              sensor_values_[sensor_0_elevation_idx])
        << "Mismatch for sensor 0, base station " << base_station_id
        << ", elevation";

    ASSERT_EQ(msg.azimuth_1[base_station_id],
              sensor_values_[sensor_1_azimuth_idx])
        << "Mismatch for sensor 1, base station " << base_station_id
        << ", azimuth";

    ASSERT_EQ(msg.elevation_1[base_station_id],
              sensor_values_[sensor_1_elevation_idx])
        << "Mismatch for sensor 1, base station " << base_station_id
        << ", elevation";

    ASSERT_EQ(msg.azimuth_2[base_station_id],
              sensor_values_[sensor_2_azimuth_idx])
        << "Mismatch for sensor 2, base station " << base_station_id
        << ", azimuth";

    ASSERT_EQ(msg.elevation_2[base_station_id],
              sensor_values_[sensor_2_elevation_idx])
        << "Mismatch for sensor 2, base station " << base_station_id
        << ", elevation";

    ASSERT_EQ(msg.azimuth_3[base_station_id],
              sensor_values_[sensor_3_azimuth_idx])
        << "Mismatch for sensor 3, base station " << base_station_id
        << ", azimuth";

    ASSERT_EQ(msg.elevation_3[base_station_id],
              sensor_values_[sensor_3_elevation_idx])
        << "Mismatch for sensor 3, base station " << base_station_id
        << ", elevation";
  }
}
