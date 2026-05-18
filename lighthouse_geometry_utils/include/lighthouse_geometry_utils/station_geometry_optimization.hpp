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

#ifndef LIGHTHOUSE_GEOMETRY_UTILS__STATION_GEOMETRY_OPTIMIZATION_HPP_
#define LIGHTHOUSE_GEOMETRY_UTILS__STATION_GEOMETRY_OPTIMIZATION_HPP_

#include "lighthouse_geometry_utils/station_geometry_initialization.hpp"

#include <Eigen/Core>

#include <array>
#include <map>
#include <vector>

#include "lighthouse_geometry_utils/datatypes.hpp"
#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils
{

/**
 * @brief Optimizes Lighthouse base station geometry using nonlinear
 * optimization.
 *
 * Uses Ceres solver to refine the base station pose estimate through
 * nonlinear optimization based on sensor measurements.
 */
class StationGeometryOptimization
{
public:
  /// Constructs an optimizer with default Lighthouse Deck sensor configuration.
  StationGeometryOptimization();

  /**
   * @brief Adds a measurement sample for optimization.
   *
   * Samples with the same deck_pose_id are considered to be taken from the
   * same deck location, possibly observing different base stations.
   *
   * @param elevations Array of 4 elevation angles in radians (angle above
   * horizontal).
   * @param azimuths Array of 4 azimuth angles in radians (horizontal angle).
   * @param station_id Identifier of the base station that produced this
   * measurement.
   * @param deck_pose_id Identifier for the deck pose where this sample was
   * taken.
   */
  void addSample(
    const std::array<double, 4> & elevations,
    const std::array<double, 4> & azimuths, StationId station_id,
    DeckPoseId deck_pose_id);

  /// @brief Clears all collected measurement samples.
  void reset();

  /**
   * @brief Solves for optimized base station poses using collected samples.
   *
   * Initial pose estimates are obtained from the internal initializer.
   * Deck IDs are automatically assigned indices based on their order of
   * appearance in the samples.
   *
   * @return Result containing optimized base station poses and their IDs.
   */
  StationPoseEstimates solve();

private:
  /// Measurement sample containing sensor angles and base station identifier.
  struct Sample
  {
    /// Array of 4 elevation angles in radians (one per sensor).
    std::array<double, 4> elevations;
    /// Array of 4 azimuth angles in radians (one per sensor).
    std::array<double, 4> azimuths;
    /// Identifier of the base station that produced this measurement.
    StationId station_id;
  };

  struct SampleLocationData
  {
    /// Collection of samples from different stations at this deck location.
    std::vector<Sample> station_data;
  };

  /// Initializer used to compute initial pose estimates for the optimizer.
  StationGeometryInitialization initializer_;

  /// 2D positions of the four sensors in the deck frame (in meters).
  std::array<Eigen::Vector2d, 4> sensor_poses_;

  /// Collection of measurement samples indexed by sample pose ID.
  /// Samples with the same pose ID were taken from the same deck location.
  std::map<DeckPoseId, SampleLocationData> deck_pose_data_;

  /// Station IDs encountered during sample collection.
  std::vector<StationId> station_ids_;

  /// Huber loss delta for bearing vector residuals.
  static constexpr double kHuberDeltaStations = 10e-3;
};

}  // namespace lighthouse_geometry_utils

#endif  // LIGHTHOUSE_GEOMETRY_UTILS__STATION_GEOMETRY_OPTIMIZATION_HPP_
