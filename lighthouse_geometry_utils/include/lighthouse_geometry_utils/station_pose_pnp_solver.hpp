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

#ifndef LIGHTHOUSE_GEOMETRY_UTILS__STATION_POSE_PNP_SOLVER_HPP_
#define LIGHTHOUSE_GEOMETRY_UTILS__STATION_POSE_PNP_SOLVER_HPP_

#include <Eigen/Core>

#include <array>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils
{

/**
 * @brief Solves for Lighthouse station pose using Perspective-n-Point
 * (PnP) algorithm.
 *
 * Computes the 6-DOF pose of a Lighthouse station relative to a tracker
 * deck frame from measured elevation and azimuth angles at four sensors.
 */
class StationPosePnPSolver
{
public:
  /// Constructs a PnP solver with default Lighthouse Deck sensor configuration.
  StationPosePnPSolver();

  /**
   * @brief Computes the station pose in the deck coordinate frame.
   *
   * @param elevations Array of 4 elevation angles in radians (angle above
   * horizontal).
   * @param azimuths Array of 4 azimuth angles in radians (horizontal angle).
   * @return SE3 transformation of the  station pose in the deck frame.
   */
  Sophus::SE3d solve(
    const std::array<double, 4> & elevations,
    const std::array<double, 4> & azimuths) const;

private:
  /// Virtual camera intrinsic matrix (3x3) for angle-to-image projection.
  cv::Mat camera_matrix_;

  /// Lens distortion coefficients (1x5, zero for Lighthouse system).
  cv::Mat distortion_coefficients_;

  /// Distance of the virtual image plane from the camera origin in meters.
  static constexpr double kVirtualPlaneDistanceMeters = 1.0;

  /**
   * @brief Projects angle measurements onto a virtual 2D image plane.
   *
   * @param elevations Array of 4 elevation angles in radians.
   * @param azimuths Array of 4 azimuth angles in radians.
   * @return Array of 4 points in the virtual image plane.
   */
  std::array<cv::Point2d, 4>
  projectToVirtualPlane(
    const std::array<double, 4> & elevations,
    const std::array<double, 4> & azimuths) const;
};

}  // namespace lighthouse_geometry_utils

#endif  // LIGHTHOUSE_GEOMETRY_UTILS__STATION_POSE_PNP_SOLVER_HPP_
