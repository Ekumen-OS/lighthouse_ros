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

#ifndef LIGHTHOUSE_DECK_UTILS__UTILS_HPP_
#define LIGHTHOUSE_DECK_UTILS__UTILS_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <sophus/se3.hpp>
#include "lighthouse_geometry_utils/datatypes.hpp"

namespace lighthouse_deck_utils
{

/**
 * @brief Load station poses from a CSV file.
 *
 * The CSV file format is: station_id,x,y,z,qx,qy,qz,qw
 * where the first line is a header and subsequent lines contain station data.
 * This format matches the output from the lighthouse_station_mapper save functionality.
 *
 * Logs errors and warnings using ROS logging.
 *
 * @param filepath Path to the CSV file.
 * @return Optional pair of (station_poses, station_ids) if successful, std::nullopt otherwise.
 */
std::optional<std::pair<
    std::vector<Sophus::SE3d>,
    std::vector<lighthouse_geometry_utils::StationId>>>
load_stations_map(const std::string & filepath);

/**
 * @brief Save station poses to a CSV file.
 *
 * The CSV file format is: station_id,x,y,z,qx,qy,qz,qw
 * where the first line is a header and subsequent lines contain station data.
 *
 * Logs errors and info using ROS logging.
 *
 * @param filepath Path to the CSV file to create.
 * @param station_poses Vector of SE3 poses to save.
 * @param station_ids Vector of station IDs corresponding to the poses.
 * @return True if saved successfully, false otherwise.
 */
bool save_stations_map(
  const std::string & filepath,
  const std::vector<Sophus::SE3d> & station_poses,
  const std::vector<lighthouse_geometry_utils::StationId> & station_ids);

}  // namespace lighthouse_deck_utils

#endif  // LIGHTHOUSE_DECK_UTILS__UTILS_HPP_
