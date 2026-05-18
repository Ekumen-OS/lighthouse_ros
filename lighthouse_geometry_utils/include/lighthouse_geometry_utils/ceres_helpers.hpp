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

#ifndef LIGHTHOUSE_GEOMETRY_UTILS__CERES_HELPERS_HPP_
#define LIGHTHOUSE_GEOMETRY_UTILS__CERES_HELPERS_HPP_

#include <ceres/ceres.h>
#include <Eigen/Dense>

#include <array>
#include <cmath>

#include <sophus/se3.hpp>

#include "lighthouse_geometry_utils/utils.hpp"

namespace lighthouse_geometry_utils
{

/// @brief Create a local parameterization for SE3 optimization
/// SE3 has 7 parameters (quaternion + translation) but 6 degrees of freedom
ceres::LocalParameterization * createSE3Parameterization();

/// Ceres cost functor for measuring bearing vector errors between observed and
/// estimated sensor positions.
struct BearingVectorErrorFunctor
{
  /**
   * @brief Constructs a functor with observed elevation and azimuth angles.
   *
   * Stores raw angles so that per-station bias offsets can be applied during
   * optimization before computing bearing vectors.
   *
   * @param elevations Array of 4 elevation angles in radians.
   * @param azimuths Array of 4 azimuth angles in radians.
   */
  BearingVectorErrorFunctor(
    const std::array<double, 4> & elevations,
    const std::array<double, 4> & azimuths)
  : elevations_(elevations), azimuths_(azimuths) {}

  /**
   * @brief Computes residuals for the optimization.
   *
   * Converts observed angles to bearing vectors, then uses chordal distance
   * on the unit sphere: r = p_hat - v_obs.
   * This keeps 3 residuals per sensor (2 effective DOF per bearing) and
   * provides a smooth, globally better-behaved gradient when angular errors
   * are large.
   *
   * @param deck_params Raw parameter array for deck SE3 pose (7).
   * @param station_params Raw parameter array for station SE3 pose (7).
   * @param residuals Output array of 12 residuals (3 per sensor × 4 sensors).
   * @return true if computation succeeded.
   */
  template<typename T>
  bool operator()(
    T const * const deck_params, T const * const station_params,
    T * residuals) const
  {
    // Map raw parameters to Sophus SE3 (poses in world frame)
    Eigen::Map<Sophus::SE3<T> const> const deck_pose_in_world(deck_params);
    Eigen::Map<Sophus::SE3<T> const> const station_pose_in_world(
      station_params);

    // Transform from deck frame to station frame
    const Sophus::SE3<T> deck_pose_in_station =
      station_pose_in_world.inverse() * deck_pose_in_world;

    for (std::size_t i = 0; i < 4; ++i) {
      // Use observed angles directly
      const T el = T(elevations_[i]);
      const T az = T(azimuths_[i]);

      // Convert corrected spherical coordinates to bearing vector
      const T cos_el = ceres::cos(el);
      Eigen::Matrix<T, 3, 1> observed_bearing(
        cos_el * ceres::cos(az), cos_el * ceres::sin(az), ceres::sin(el));
      observed_bearing.normalize();

      // Sensor position in deck frame (z = 0 for planar sensors)
      Eigen::Matrix<T, 3, 1> sensor_position_in_deck(
        T(kLighthouseDeckSensorPoses[i][0]),
        T(kLighthouseDeckSensorPoses[i][1]),
        T(0.0));

      // Transform sensor to station frame
      const Eigen::Matrix<T, 3, 1> sensor_position_in_station =
        deck_pose_in_station * sensor_position_in_deck;

      const Eigen::Matrix<T, 3, 1> estimated_bearing =
        sensor_position_in_station.normalized();

      const Eigen::Matrix<T, 3, 1> chordal_residual =
        estimated_bearing - observed_bearing;

      residuals[i * 3 + 0] = chordal_residual[0];
      residuals[i * 3 + 1] = chordal_residual[1];
      residuals[i * 3 + 2] = chordal_residual[2];
    }

    return true;
  }

private:
  /// Raw observed elevation angles in radians.
  std::array<double, 4> elevations_;
  /// Raw observed azimuth angles in radians.
  std::array<double, 4> azimuths_;
};

/// Ceres cost functor that penalizes non-zero bias offsets,
/// pushing them toward zero during optimization.
struct BiasRegularizationFunctor
{
  template<typename T>
  bool operator()(T const * const biases, T * residuals) const
  {
    for (std::size_t i = 0; i < 8; ++i) {
      residuals[i] = biases[i];
    }
    return true;
  }
};

}  // namespace lighthouse_geometry_utils

#endif  // LIGHTHOUSE_GEOMETRY_UTILS__CERES_HELPERS_HPP_
