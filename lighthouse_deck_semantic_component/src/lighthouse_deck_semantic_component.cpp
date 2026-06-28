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

#include "lighthouse_deck_semantic_component/lighthouse_deck_semantic_component.hpp"

#include <iomanip>
#include <sstream>

namespace lighthouse_deck_semantic_component {

LighthouseDeckSemanticComponent::LighthouseDeckSemanticComponent(
    const std::string &name)
    : SemanticComponentInterface(name, NUM_INTERFACES) {
  for (size_t sensor = 0; sensor < NUM_SENSORS; ++sensor) {
    for (size_t base_station = 0; base_station < NUM_BASE_STATIONS;
         ++base_station) {
      const std::string interface_prefix = name_ + "_sensor_" +
                                           std::to_string(sensor) + "_base_" +
                                           format_base_station_id(base_station);

      interface_names_.emplace_back(interface_prefix + "/" + "azimuth");
      interface_names_.emplace_back(interface_prefix + "/" + "elevation");
    }
  }

  for (size_t base_station = 0; base_station < NUM_BASE_STATIONS;
       ++base_station) {
    const std::string interface_name = name_ + "_base_" +
                                       format_base_station_id(base_station) +
                                       "/" + "timestamp";
    interface_names_.emplace_back(interface_name);
  }

  for (auto &base_station : base_station_data_) {
    for (auto &sensor : base_station.sensors) {
      sensor.azimuth = std::numeric_limits<double>::quiet_NaN();
      sensor.elevation = std::numeric_limits<double>::quiet_NaN();
    }
    base_station.timestamp = std::numeric_limits<double>::quiet_NaN();
  }
}

std::string LighthouseDeckSemanticComponent::format_base_station_id(
    size_t base_station_id) const {
  std::ostringstream oss;
  oss << std::setw(2) << std::setfill('0') << base_station_id;
  return oss.str();
}

void LighthouseDeckSemanticComponent::update_from_interfaces() {
  size_t interface_idx = 0;
  for (size_t sensor = 0; sensor < NUM_SENSORS; ++sensor) {
    for (size_t base_station = 0; base_station < NUM_BASE_STATIONS;
         ++base_station) {
      base_station_data_[base_station].sensors[sensor].azimuth =
          state_interfaces_[interface_idx++].get().get_value();
      base_station_data_[base_station].sensors[sensor].elevation =
          state_interfaces_[interface_idx++].get().get_value();
    }
  }

  for (size_t base_station = 0; base_station < NUM_BASE_STATIONS;
       ++base_station) {
    base_station_data_[base_station].timestamp =
        state_interfaces_[interface_idx++].get().get_value();
  }
}

std::array<LighthouseDeckSemanticComponent::BaseStationData, NUM_BASE_STATIONS>
LighthouseDeckSemanticComponent::get_base_station_data() {
  update_from_interfaces();
  return base_station_data_;
}

} // namespace lighthouse_deck_semantic_component
