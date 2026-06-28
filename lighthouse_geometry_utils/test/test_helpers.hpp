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

#include <ceres/ceres.h>

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
/// @return A pair of arrays containing [elevations, azimuths] for each sensor
std::pair<std::array<double, 4>, std::array<double, 4>>
compute_expected_measurements(
  const Sophus::SE3d & base_station_pose_in_deck_frame);

/// @brief Compute expected measurements with added Gaussian noise
/// @param base_station_pose_in_deck_frame The pose of the base station in the
/// deck frame
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

/// @brief Create an upright lighthouse station pose
/// @param position The position of the station
/// @param target A point in the deck area to orient toward
/// @return SE3 pose with Z-axis pointing up and X-axis pointing toward target
/// @details Creates a realistic lighthouse station mounting where Z-axis is
///          vertical (upright) and X-axis points horizontally toward the target.
///          This represents typical ceiling or wall-mounted configurations.
Sophus::SE3d create_upright_station_pose(
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

/**
 * @brief Ceres cost functor for calculating V2 sweep plane intersection timestamps.
 *
 * This functor models the forward problem of a V2 Lighthouse base station:
 * - A rotor with TWO tilted sweep planes (at ±30° physical tilt)
 * - Both planes on the same rotor, separated by 120° (2π/3 radians)
 * - A deck moving with constant velocity
 * - Four sensors on the deck at known positions
 *
 * The functor calculates when each tilted sweep plane intersects each sensor
 * as the deck moves through space and the rotor sweeps both planes.
 *
 * Physical model (V2 station):
 * - First sweep: physical tilt = -30° (-π/6), at rotor phase φ
 * - Second sweep: physical tilt = +30° (+π/6), at rotor phase φ + 2π/3
 * - The plane rotates around a horizontal axis (parallel to station mounting)
 * - When a sensor point lies on the plane, it detects the light
 *
 * Parameters to optimize:
 * - sensor_timestamps[8]: Timestamps (seconds) when each sensor is hit
 *   - [0-3]: First sweep (tilt -π/6), sensors 0-3
 *   - [4-7]: Second sweep (tilt +π/6), sensors 0-3
 *
 * Fixed inputs (stored in functor):
 * - t0: Reference timestamp (seconds) defining the phase origin (at t=t0, phase=0)
 * - rotor_period: Period of one full rotation (seconds, ~20ms for V2)
 * - deck_pose_at_t0: Initial pose of deck at time t0
 * - deck_velocity: Linear velocity of deck (m/s) in world frame
 * - station_pose: Pose of the station in world frame
 *
 * Rotor phase convention (raw):
 * - At t = t0: rotor phase = 0 (rotor X-axis pointing backward, -X direction)
 * - At t = t0 + period/4: rotor phase = π/2 (pointing left, -Y direction)
 * - At t = t0 + period/2: rotor phase = π (pointing forward, +X direction)
 * - At t = t0 + 3*period/4: rotor phase = 3π/2 (pointing right, +Y direction)
 *
 * Note: The raw phase uses [0, 2π] range. When comparing with sensor angles,
 * the phase is shifted by π to match the forward-centered convention where 0 = forward.
 *
 * Constants (class members):
 * - kSensorXDistance: Sensor spacing in X direction (0.0300m)
 * - kSensorYDistance: Sensor spacing in Y direction (0.0150m)
 * - kFirstSweepTilt: Physical tilt of first sweep plane (-π/6 for V2)
 * - kSecondSweepTilt: Physical tilt of second sweep plane (+π/6 for V2)
 * - kTiltOffset: Additional calibration offset (0.0 for standard V2)
 */
struct SweepPlaneTimestampFunctor
{
  // Standard Lighthouse deck sensor layout
  static constexpr double kSensorXDistance = 0.0300;  // 30.0mm
  static constexpr double kSensorYDistance = 0.0150;  // 15.0mm

  // Standard V2 sweep plane tilts
  static constexpr double kFirstSweepTilt = -M_PI / 6.0;   // -30°
  static constexpr double kSecondSweepTilt = M_PI / 6.0;   // +30°
  static constexpr double kTiltOffset = 0.0;               // No offset

  // Standard sensor positions in deck frame (2D)
  inline static const std::array<Eigen::Vector2d, 4> kSensorPositions = {{
    {-kSensorXDistance / 2, +kSensorYDistance / 2},  // top-left
    {-kSensorXDistance / 2, -kSensorYDistance / 2},  // bottom-left
    {+kSensorXDistance / 2, +kSensorYDistance / 2},  // top-right
    {+kSensorXDistance / 2, -kSensorYDistance / 2}   // bottom-right
  }};
  /**
   * @brief Construct the functor with fixed parameters.
   *
   * @param t0 Reference timestamp (seconds) defining time origin for phase calculation
   * @param rotor_period Period of full rotation (seconds), ~20ms for V2
   * @param deck_pose_at_t0 Initial deck pose in world frame at time t0
   * @param deck_velocity Deck linear velocity (m/s) in world frame
   * @param station_pose Station pose in world frame
   *
   * Note: At t = t0, the raw rotor phase = 0. The rotor X-axis points backward (-X)
   * at phase 0, and forward (+X) at phase π. The forward-centered transform is applied
   * during residual calculation.
   */
  SweepPlaneTimestampFunctor(
    double t0,
    double rotor_period,
    const Sophus::SE3d & deck_pose_at_t0,
    const Eigen::Vector3d & deck_velocity,
    const Sophus::SE3d & station_pose)
  : t0_(t0),
    rotor_period_(rotor_period),
    deck_pose_at_t0_(deck_pose_at_t0),
    deck_velocity_(deck_velocity),
    station_pose_(station_pose)
  {}

  /**
   * @brief Compute residuals for both sweep plane intersections.
   *
   * For each sensor and each sweep, computes the residual between the sensor's
   * position on the sweep plane and zero. When the residual is zero, the sensor
   * lies exactly on the sweep plane at the given timestamp.
   *
   * V2 rotor geometry:
   * - First sweep (tilt -π/6) at phase φ
   * - Second sweep (tilt +π/6) at phase φ + 2π/3 (physically mounted 120° ahead)
   *
   * The sweep plane at time t has:
   * - Phase angle: φ(t) = 2π * (t - t0) / period (same formula for both sweeps)
   * - The 120° physical separation produces timestamps differing by ~period/3
   * - The rotor_offset (±π/3) in target_phase compensates for what calculateV2Angles applies
   * - Normal vector (in station frame):
   *   n = [sin(φ) * cos(tilt), -cos(φ) * cos(tilt), sin(tilt)]
   *   where tilt = sweep_tilt + tilt_offset
   *
   * A point p is on the plane when: n · (p - station_origin) = 0
   *
   * @param sensor_timestamps Array of 8 timestamps to optimize:
   *        [0-3]: first sweep (sensors 0-3), [4-7]: second sweep (sensors 0-3)
   * @param residuals Output array of 8 residuals (one per timestamp)
   * @return true if computation succeeded
   */
  template<typename T>
  bool operator()(T const * const sensor_timestamps, T * residuals) const
  {
    // Station origin and rotation in world frame
    const Eigen::Matrix<T, 3, 1> station_origin =
      station_pose_.translation().template cast<T>();
    const Eigen::Matrix<T, 3, 3> station_rotation =
      station_pose_.so3().matrix().template cast<T>();

    // Process both sweeps
    for (int sweep_idx = 0; sweep_idx < 2; ++sweep_idx) {
      // Select tilt for this sweep
      const T sweep_tilt = sweep_idx == 0 ?
        T(kFirstSweepTilt) : T(kSecondSweepTilt);
      const T total_tilt = sweep_tilt + T(kTiltOffset);

      // Rotor offsets to compensate for calculateV2Angles applying ±π/3
      // Sweep 0: calculateV2Angles does phase - π + π/3, so we need offset = -π/3
      // Sweep 1: calculateV2Angles does phase - π - π/3, so we need offset = +π/3
      const T rotor_offset = sweep_idx == 0 ? T(-M_PI / 3.0) : T(M_PI / 3.0);

      // Process each sensor for this sweep
      for (std::size_t sensor_idx = 0; sensor_idx < 4; ++sensor_idx) {
        const std::size_t timestamp_idx = sweep_idx * 4 + sensor_idx;

        // Timestamp for this sensor in this sweep
        const T t = sensor_timestamps[timestamp_idx];
        const T dt = t - T(t0_);

        // Rotor phase at time t: raw angle in [0, 2π] convention
        // At t = t0 (dt = 0): phase = 0 (rotor pointing backward, -X direction)
        // At dt = period/2: phase = π (rotor pointing forward, +X direction)
        // Same calculation for both sweeps - the 120° separation is handled
        // by rotor_offset in target_phase, matching what calculateV2Angles applies
        const T rotor_phase = T(2.0 * M_PI) * dt / T(rotor_period_);

        // Deck pose at time t (with constant velocity motion)
        // position(t) = position(t0) + velocity * dt
        const Eigen::Matrix<T, 3, 1> deck_position_at_t =
          deck_pose_at_t0_.translation().template cast<T>() +
          deck_velocity_.template cast<T>() * dt;

        // Deck rotation (assuming no angular velocity for now)
        const Eigen::Matrix<T, 3, 3> deck_rotation =
          deck_pose_at_t0_.so3().matrix().template cast<T>();

        // Sensor position in deck frame (z = 0 for planar deck)
        const Eigen::Matrix<T, 3, 1> sensor_in_deck(
          T(kSensorPositions[sensor_idx].x()),
          T(kSensorPositions[sensor_idx].y()),
          T(0.0));

        // Transform sensor to world frame
        const Eigen::Matrix<T, 3, 1> sensor_in_world =
          deck_position_at_t + deck_rotation * sensor_in_deck;

        // Vector from station origin to sensor
        const Eigen::Matrix<T, 3, 1> station_to_sensor =
          sensor_in_world - station_origin;

        // Transform sensor position to station frame for α_p calculation
        const Eigen::Matrix<T, 3, 1> sensor_in_station =
          station_rotation.transpose() * station_to_sensor;

        const T x = sensor_in_station(0);
        const T y = sensor_in_station(1);
        const T z = sensor_in_station(2);

        // Calculate predicted v2_angle α_p where this tilted plane intersects XY plane
        // Based on Crazyflie measurement model:
        // α_s = atan2(y, x) - azimuth to sensor (forward-centered: 0 = +X)
        // α_t = -arcsin(z·tan(tilt) / r) - tilt correction
        // α_p = α_s - α_t - predicted v2_angle (forward-centered: 0 = +X)
        const T r = ceres::sqrt(x * x + y * y);
        const T alpha_s = ceres::atan2(y, x);
        const T alpha_t = -ceres::asin(z * ceres::tan(total_tilt) / r);
        const T alpha_p = alpha_s - alpha_t;

        // Convention alignment:
        // - rotor_phase: raw phase convention (0 = backward, π = forward)
        // - α_p: v2_angle convention (0 = forward, from atan2)
        // - calculateV2Angles does: v2_angle = raw_phase - π ± π/3
        //
        // To compare: convert α_p (forward-centered) to raw phase (backward-centered)
        // target_phase = α_p + π (convention shift) + rotor_offset (±π/3 correction)
        //
        // Use atan2(sin, cos) to compute the angular difference automatically normalized to [-π, π]
        const T target_phase = alpha_p + T(M_PI) + rotor_offset;
        const T phase_diff = ceres::atan2(
          ceres::sin(rotor_phase - target_phase),
          ceres::cos(rotor_phase - target_phase)
        );

        residuals[timestamp_idx] = phase_diff;
      }
    }

    return true;
  }

private:
  double t0_;                      ///< Reference time (when raw phase = 0)
  double rotor_period_;            ///< Rotor period (seconds)
  Sophus::SE3d deck_pose_at_t0_;   ///< Initial deck pose
  Eigen::Vector3d deck_velocity_;  ///< Deck velocity (m/s)
  Sophus::SE3d station_pose_;      ///< Station pose in world
};

/**
 * @brief Helper function to solve for V2 sensor hit timestamps using the sweep plane model.
 *
 * This function sets up and solves a Ceres optimization problem to find the
 * timestamps when each of the four sensors is hit by both sweep planes of a V2 station.
 *
 * V2 stations have two tilted planes on the same rotor:
 * - First sweep: tilt -π/6 (-30°) at rotor phase φ
 * - Second sweep: tilt +π/6 (+30°) at rotor phase φ + 2π/3 (120° apart)
 *
 * Example usage for V2 station simulation:
 * @code
 *   // V2 station parameters
 *   const double rotor_period = 0.01998;  // ~20ms for V2 channel 0
 *
 *   // Initial conditions at t0 = 0.0
 *   // At t0, the raw rotor phase = 0 (rotor pointing backward)
 *   // At t0 + period/2, phase = π (rotor pointing forward)
 *   Sophus::SE3d deck_pose = ...; // Deck position and orientation
 *   Eigen::Vector3d deck_velocity(0.1, 0.0, 0.0);  // Moving 10 cm/s in X
 *   Sophus::SE3d station_pose = ...; // Station position and orientation
 *
 *   // Initial guess for timestamps (around t0)
 *   // [0-3]: first sweep, [4-7]: second sweep
 *   std::array<double, 8> timestamps = {
 *     0.001, 0.001, 0.001, 0.001,  // first sweep guesses
 *     0.011, 0.011, 0.011, 0.011   // second sweep guesses (~half period later)
 *   };
 *
 *   // Solve for actual intersection timestamps
 *   // (sensor positions, tilts, and offsets are built-in constants)
 *   bool success = solve_sweep_plane_timestamps(
 *     0.0,              // t0
 *     rotor_period,
 *     deck_pose,
 *     deck_velocity,
 *     station_pose,
 *     timestamps        // output: optimized timestamps
 *   );
 *
 *   // Extract per-sweep timestamps
 *   auto first_sweep_ts = {timestamps[0], timestamps[1], timestamps[2], timestamps[3]};
 *   auto second_sweep_ts = {timestamps[4], timestamps[5], timestamps[6], timestamps[7]};
 *
 *   // Convert timestamps to phase offsets (for V2 protocol)
 *   for (int i = 0; i < 4; ++i) {
 *     double phase_0 = 2.0 * M_PI * timestamps[i] / rotor_period;
 *     double phase_1 = 2.0 * M_PI * timestamps[i+4] / rotor_period;
 *     // phase values can be converted to timing offsets for simulation
 *   }
 * @endcode
 *
 * @param t0 Reference timestamp (seconds) defining the time origin for phase calculation
 * @param rotor_period Rotor period (seconds)
 * @param deck_pose_at_t0 Initial deck pose at time t0
 * @param deck_velocity Deck linear velocity (m/s)
 * @param station_pose Station pose in world frame
 * @param[in,out] sensor_timestamps Initial guess and output optimized timestamps (8 values)
 *                                   [0-3]: first sweep, [4-7]: second sweep
 * @return true if optimization converged successfully
 *
 * Note: The rotor phase is calculated as 2π(t-t0)/period, giving raw phase in [0, 2π]
 * where phase=0 is backward and phase=π is forward. The forward-centered transform
 * (adding π) is applied during residual calculation to match calculateV2Angles convention.
 */
bool solve_sweep_plane_timestamps(
  double t0,
  double rotor_period,
  const Sophus::SE3d & deck_pose_at_t0,
  const Eigen::Vector3d & deck_velocity,
  const Sophus::SE3d & station_pose,
  std::array<double, 8> & sensor_timestamps);

}  // namespace lighthouse_geometry_utils::test

#endif  // TEST_HELPERS_HPP_
