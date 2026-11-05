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

#ifndef LIGHTHOUSE_DECK_SEMANTIC_COMPONENT__LIGHTHOUSE_DECK_SEMANTIC_COMPONENT_HPP_
#define LIGHTHOUSE_DECK_SEMANTIC_COMPONENT__LIGHTHOUSE_DECK_SEMANTIC_COMPONENT_HPP_

#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "semantic_components/semantic_component_interface.hpp"

namespace lighthouse_deck_semantic_component {

constexpr size_t NUM_SENSORS = 4;
constexpr size_t NUM_BASE_STATIONS = 4;

// Each sensor has azimuth and elevation per base station, plus timestamp per
// base station
constexpr size_t NUM_INTERFACES =
    (NUM_SENSORS * NUM_BASE_STATIONS * 2) + NUM_BASE_STATIONS;

class LighthouseDeckSemanticComponent
    : public semantic_components::SemanticComponentInterface<
          std::array<double, NUM_INTERFACES>> {
public:
  struct SensorData {
    double azimuth;
    double elevation;
  };

  struct BaseStationData {
    std::array<SensorData, NUM_SENSORS> sensors;
    double timestamp;
  };

  /**
   * \param[in] name The name of the lighthouse deck component (e.g.,
   * "lighthouse")
   */
  explicit LighthouseDeckSemanticComponent(const std::string &name);

  virtual ~LighthouseDeckSemanticComponent() = default;

  /**
   * \return Copy of the base station data array
   */
  std::array<BaseStationData, NUM_BASE_STATIONS> get_base_station_data();

protected:
  std::string format_base_station_id(size_t base_station_id) const;

  std::array<BaseStationData, NUM_BASE_STATIONS> base_station_data_;

  void update_from_interfaces();
};

} // namespace lighthouse_deck_semantic_component

#endif // LIGHTHOUSE_DECK_SEMANTIC_COMPONENT__LIGHTHOUSE_DECK_SEMANTIC_COMPONENT_HPP_
