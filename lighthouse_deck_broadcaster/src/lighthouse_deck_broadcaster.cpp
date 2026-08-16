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

#include "lighthouse_deck_broadcaster/lighthouse_deck_broadcaster.hpp"

#include <memory>
#include <string>

namespace lighthouse_deck_broadcaster {

LighthouseDeckBroadcaster::LighthouseDeckBroadcaster()
    : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn LighthouseDeckBroadcaster::on_init() {
  try {
    auto_declare<std::string>("device_name", "");
    auto_declare<std::string>("frame_id", "lighthouse_deck");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exception thrown during init stage with message: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LighthouseDeckBroadcaster::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  device_name_ = get_node()->get_parameter("device_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();

  if (device_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'device_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  try {
    lighthouse_deck_ = std::make_unique<
        lighthouse_deck_semantic_component::LighthouseDeckSemanticComponent>(
        device_name_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exception thrown during configure stage with message: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  using LighthouseDeckMeasurement =
      lighthouse_deck_msgs::msg::LighthouseDeckMeasurement;
  auto publisher = get_node()->create_publisher<LighthouseDeckMeasurement>(
      "lighthouse", rclcpp::SystemDefaultsQoS());

  realtime_publisher_ = std::make_shared<StatePublisher>(publisher);

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured LighthouseDeckBroadcaster for '%s'",
              device_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
LighthouseDeckBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
LighthouseDeckBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = lighthouse_deck_->get_state_interface_names();

  return state_interfaces_config;
}

controller_interface::CallbackReturn LighthouseDeckBroadcaster::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  lighthouse_deck_->assign_loaned_state_interfaces(state_interfaces_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LighthouseDeckBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  lighthouse_deck_->release_interfaces();
  realtime_publisher_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LighthouseDeckBroadcaster::update(const rclcpp::Time &time,
                                  const rclcpp::Duration & /*period*/) {
  auto base_station_data = lighthouse_deck_->get_base_station_data();

  if (realtime_publisher_->trylock()) {
    auto &msg = realtime_publisher_->msg_;

    msg.header.stamp = time;
    msg.header.frame_id = frame_id_;

    // Clear all vectors
    msg.station_id.clear();
    msg.azimuth_0.clear();
    msg.azimuth_1.clear();
    msg.azimuth_2.clear();
    msg.azimuth_3.clear();
    msg.elevation_0.clear();
    msg.elevation_1.clear();
    msg.elevation_2.clear();
    msg.elevation_3.clear();

    // Populate vectors with data from all base stations
    for (size_t base_station_id = 0;
         base_station_id <
         lighthouse_deck_semantic_component::NUM_BASE_STATIONS;
         ++base_station_id) {
      msg.station_id.push_back(static_cast<int32_t>(base_station_id));
      msg.azimuth_0.push_back(
          base_station_data[base_station_id].sensors[0].azimuth);
      msg.azimuth_1.push_back(
          base_station_data[base_station_id].sensors[1].azimuth);
      msg.azimuth_2.push_back(
          base_station_data[base_station_id].sensors[2].azimuth);
      msg.azimuth_3.push_back(
          base_station_data[base_station_id].sensors[3].azimuth);
      msg.elevation_0.push_back(
          base_station_data[base_station_id].sensors[0].elevation);
      msg.elevation_1.push_back(
          base_station_data[base_station_id].sensors[1].elevation);
      msg.elevation_2.push_back(
          base_station_data[base_station_id].sensors[2].elevation);
      msg.elevation_3.push_back(
          base_station_data[base_station_id].sensors[3].elevation);
    }

    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

} // namespace lighthouse_deck_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lighthouse_deck_broadcaster::LighthouseDeckBroadcaster,
                       controller_interface::ControllerInterface)
