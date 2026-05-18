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

#ifndef LIGHTHOUSE_GEOMETRY_UTILS__DECK_POSE_OPTIMIZATION_HPP_
#define LIGHTHOUSE_GEOMETRY_UTILS__DECK_POSE_OPTIMIZATION_HPP_

#include <Eigen/Core>

#include <array>
#include <tuple>
#include <vector>

#include "lighthouse_geometry_utils/datatypes.hpp"
#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils
{

/**
 * @brief Optimizes the Lighthouse deck pose using nonlinear optimization.
 *
 * Uses Ceres solver to estimate the deck pose given known base station poses
 * and sensor measurements. All measurements are assumed to come from the
 * same deck pose.
 */
class DeckPoseOptimization
{
public:
  struct Sample
  {
    /// Array of 4 elevation angles in radians (one per sensor).
    std::array<double, 4> elevations;
    /// Array of 4 azimuth angles in radians (one per sensor).
    std::array<double, 4> azimuths;
    /// Identifier of the base station that produced this measurement.
    StationId station_id;
  };

  /**
   * @brief Constructs an optimizer with known station poses.
   *
   * @param station_poses Known SE3 poses of the base stations.
   * @param station_ids Identifiers for each station pose.
   */
  DeckPoseOptimization(
    const std::vector<Sophus::SE3d> & station_poses,
    const std::vector<StationId> & station_ids);

  /**
   * @brief Solves for the deck pose using the given measurements.
   *
   * All measurements are assumed to come from the same deck pose.
   *
   * @param samples Vector of measurement samples from different stations.
   * @return Tuple of estimated SE3 pose and autocovariance diagonal.
   */
  std::tuple<Sophus::SE3d, AutoCovDiagonal> solve(
    const std::vector<Sample> & samples) const;

private:
  /// Known base station poses (constant during optimization).
  std::vector<Sophus::SE3d> station_poses_;

  /// Station IDs corresponding to station_poses_.
  std::vector<StationId> station_ids_;

  /// 2D positions of the four sensors in the deck frame (in meters).
  std::array<Eigen::Vector2d, 4> sensor_poses_;

  /// Huber loss delta for bearing vector residuals.
  static constexpr double kHuberDeltaStations = 10e-3;
};

}  // namespace lighthouse_geometry_utils

#endif  // LIGHTHOUSE_GEOMETRY_UTILS__DECK_POSE_OPTIMIZATION_HPP_
