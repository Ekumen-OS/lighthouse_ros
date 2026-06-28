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

#ifndef LIGHTHOUSE_GEOMETRY_UTILS__DATATYPES_HPP_
#define LIGHTHOUSE_GEOMETRY_UTILS__DATATYPES_HPP_

#include <cstddef>
#include <map>
#include <vector>

#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils
{

using StationId = std::size_t;
using DeckPoseId = std::size_t;

struct AutoCovDiagonal
{
  /// Variance in x (translation) in meters².
  double x;
  /// Variance in y (translation) in meters².
  double y;
  /// Variance in z (translation) in meters².
  double z;
  /// Variance in roll (rotation) in radians².
  double roll;
  /// Variance in pitch (rotation) in radians².
  double pitch;
  /// Variance in yaw (rotation) in radians².
  double yaw;
};

/// Result of the station geometry initialization.
///
/// Contains best-guess initial estimates of station and deck poses based on
/// the collected samples. These estimates are intended to serve as starting
/// points for subsequent optimization and should not be treated as final
/// calibration results.
struct NetworkNodePoses
{
  /// Best-guess initial estimates of base station poses in the reference frame.
  std::map<StationId, Sophus::SE3d> station_poses;
  /// Best-guess initial estimates of deck poses in the reference frame.
  std::map<DeckPoseId, Sophus::SE3d> deck_poses;
};

/// Result of the station geometry optimization
struct StationPoseEstimates
{
  /// Optimized SE3 poses of the base stations.
  std::vector<Sophus::SE3d> station_poses;
  /// Identifiers corresponding to each station pose.
  std::vector<StationId> station_ids;
  /// Diagonal elements of autocovariance matrices for each station pose.
  std::vector<AutoCovDiagonal> station_autocov_diagonals;
};

}  // namespace lighthouse_geometry_utils

#endif  // LIGHTHOUSE_GEOMETRY_UTILS__DATATYPES_HPP_
