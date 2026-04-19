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

#ifndef TEST_HELPERS_HPP_
#define TEST_HELPERS_HPP_

#include <Eigen/Core>

#include <array>
#include <random>
#include <utility>
#include <vector>

#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils::test
{

/// @brief Compute expected azimuth and elevation measurements from base station
/// pose and sensor positions
/// @param base_station_pose_in_deck_frame The pose of the base station in the
/// deck frame
/// @param sensor_poses The positions of the sensors in the deck frame
/// @return A pair of arrays containing [elevations, azimuths] for each sensor
std::pair<std::array<double, 4>, std::array<double, 4>>
compute_expected_measurements(
  const Sophus::SE3d & base_station_pose_in_deck_frame,
  const std::vector<Eigen::Vector2d> & sensor_poses);

/// @brief Compute expected measurements with added Gaussian noise
/// @param base_station_pose_in_deck_frame The pose of the base station in the
/// deck frame
/// @param sensor_poses The positions of the sensors in the deck frame
/// @param elevation_noise_mean Mean of the elevation noise distribution
/// @param elevation_noise_stddev Standard deviation of the elevation noise
/// distribution
/// @param azimuth_noise_mean Mean of the azimuth noise distribution
/// @param azimuth_noise_stddev Standard deviation of the azimuth noise
/// distribution
/// @param rng Random number generator
/// @return A pair of arrays containing noisy [elevations, azimuths] for each
/// sensor
std::pair<std::array<double, 4>, std::array<double, 4>>
compute_expected_measurements_with_noise(
  const Sophus::SE3d & base_station_pose_in_deck_frame,
  const std::vector<Eigen::Vector2d> & sensor_poses,
  double elevation_noise_mean, double elevation_noise_stddev,
  double azimuth_noise_mean, double azimuth_noise_stddev, std::mt19937 & rng);

/// @brief Compute the Euclidean distance between the translations of two poses
/// @param observed_pose The observed or recovered pose
/// @param expected_pose The expected or ground truth pose
/// @return The norm of the translation difference
double compute_translation_error(
  const Sophus::SE3d & observed_pose,
  const Sophus::SE3d & expected_pose);

/// @brief Compute the maximum angular error between the X-axis and Z-axis
/// bearings of two poses
/// @param observed_pose The observed or recovered pose
/// @param expected_pose The expected or ground truth pose
/// @return The maximum angle in radians between the X-axes or Z-axes
double compute_direction_error_radians(
  const Sophus::SE3d & observed_pose,
  const Sophus::SE3d & expected_pose);

/// @brief Create a pose at the given position oriented to face a target point
/// @param position The position of the pose
/// @param target The point to look at (X-axis will point toward this)
/// @return SE3 pose with X-axis pointing from position to target
Sophus::SE3d create_pose_facing_target(
  const Eigen::Vector3d & position,
  const Eigen::Vector3d & target);

/// @brief Add random noise to a pose
/// @param pose The original pose
/// @param max_translation_noise Maximum translation noise in meters
/// @param max_rotation_noise Maximum rotation noise in radians
/// @param rng Random number generator
/// @return Noisy pose
Sophus::SE3d add_noise_to_pose(
  const Sophus::SE3d & pose,
  double max_translation_noise,
  double max_rotation_noise, std::mt19937 & rng);

}  // namespace lighthouse_geometry_utils::test

#endif  // TEST_HELPERS_HPP_
