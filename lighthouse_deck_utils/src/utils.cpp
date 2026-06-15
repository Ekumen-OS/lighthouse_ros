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

#include "lighthouse_deck_utils/utils.hpp"

#include <fstream>
#include <optional>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

namespace lighthouse_deck_utils
{

std::optional<std::pair<
    std::vector<Sophus::SE3d>,
    std::vector<lighthouse_geometry_utils::StationId>>>
load_stations_map(const std::string & filepath)
{
  auto logger = rclcpp::get_logger("load_stations_map");

  std::ifstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open stations map file: %s", filepath.c_str());
    return std::nullopt;
  }

  std::vector<Sophus::SE3d> station_poses;
  std::vector<lighthouse_geometry_utils::StationId> station_ids;

  std::string line;
  // Skip header line
  if (!std::getline(file, line)) {
    RCLCPP_ERROR(logger, "Empty stations map file: %s", filepath.c_str());
    return std::nullopt;
  }

  // Check header format
  if (line.find("station_id") == std::string::npos) {
    RCLCPP_ERROR(logger, "Invalid header in stations map file: %s", filepath.c_str());
    return std::nullopt;
  }

  // Read data lines
  int line_number = 1;
  while (std::getline(file, line)) {
    ++line_number;

    // Skip empty lines
    if (line.empty()) {
      continue;
    }

    std::istringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;

    // Parse CSV line
    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    // Expect 8 values: station_id, x, y, z, qx, qy, qz, qw
    if (tokens.size() != 8) {
      RCLCPP_WARN(
        logger,
        "Invalid line %d in stations map file (expected 8 values, got %zu)",
        line_number, tokens.size());
      continue;
    }

    try {
      // Parse station ID
      lighthouse_geometry_utils::StationId station_id = std::stoul(tokens[0]);

      // Parse translation
      double x = std::stod(tokens[1]);
      double y = std::stod(tokens[2]);
      double z = std::stod(tokens[3]);
      Eigen::Vector3d translation(x, y, z);

      // Parse quaternion (file format: qx, qy, qz, qw)
      double qx = std::stod(tokens[4]);
      double qy = std::stod(tokens[5]);
      double qz = std::stod(tokens[6]);
      double qw = std::stod(tokens[7]);
      Eigen::Quaterniond quaternion(qw, qx, qy, qz);

      // Create SE3 pose
      Sophus::SE3d se3_pose(quaternion, translation);

      // Store pose and ID
      station_poses.push_back(se3_pose);
      station_ids.push_back(station_id);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        logger,
        "Error parsing line %d in stations map file: %s",
        line_number, e.what());
      continue;
    }
  }

  if (station_poses.empty()) {
    RCLCPP_ERROR(logger, "No valid station poses found in file: %s", filepath.c_str());
    return std::nullopt;
  }

  RCLCPP_INFO(
    logger,
    "Successfully loaded %zu station poses from %s",
    station_poses.size(), filepath.c_str());

  return std::make_pair(std::move(station_poses), std::move(station_ids));
}

bool save_stations_map(
  const std::string & filepath,
  const std::vector<Sophus::SE3d> & station_poses,
  const std::vector<lighthouse_geometry_utils::StationId> & station_ids)
{
  auto logger = rclcpp::get_logger("save_stations_map");

  // Validate input
  if (station_poses.size() != station_ids.size()) {
    RCLCPP_ERROR(
      logger,
      "Station poses and IDs size mismatch: %zu poses vs %zu ids",
      station_poses.size(), station_ids.size());
    return false;
  }

  if (station_poses.empty()) {
    RCLCPP_ERROR(logger, "No station poses to save");
    return false;
  }

  std::ofstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open file for writing: %s", filepath.c_str());
    return false;
  }

  // Write header
  file << "station_id,x,y,z,qx,qy,qz,qw\n";
  file << std::fixed << std::setprecision(9);

  // Write station data
  for (std::size_t i = 0; i < station_ids.size(); ++i) {
    const auto & pose = station_poses[i];
    const auto t_vec = pose.translation();
    const auto q = pose.unit_quaternion();
    file
      << station_ids[i] << ","
      << t_vec.x() << "," << t_vec.y() << "," << t_vec.z() << ","
      << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
  }

  // Check for write errors
  if (!file) {
    RCLCPP_ERROR(logger, "Error while writing file: %s", filepath.c_str());
    return false;
  }

  RCLCPP_INFO(
    logger, "Station map saved to %s with %zu stations",
    filepath.c_str(), station_ids.size());
  return true;
}

}  // namespace lighthouse_deck_utils
