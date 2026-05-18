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

#include "test_helpers.hpp"

#include <ceres/ceres.h>

#include <algorithm>
#include <cmath>

#include "lighthouse_geometry_utils/utils.hpp"

namespace lighthouse_geometry_utils::test
{

std::pair<std::array<double, 4>, std::array<double, 4>>
compute_expected_measurements(
  const Sophus::SE3d & base_station_pose_in_deck_frame)
{
  std::array<double, 4> elevations;
  std::array<double, 4> azimuths;

  const Sophus::SE3d deck_pose_in_base_frame =
    base_station_pose_in_deck_frame.inverse();

  for (std::size_t i = 0; i < kLighthouseDeckSensorPoses.size(); ++i) {
    const Eigen::Vector3d sensor_in_deck(kLighthouseDeckSensorPoses[i].x(),
      kLighthouseDeckSensorPoses[i].y(), 0.0);

    const Eigen::Vector3d sensor_in_base =
      deck_pose_in_base_frame * sensor_in_deck;

    const double x = sensor_in_base.x();
    const double y = sensor_in_base.y();
    const double z = sensor_in_base.z();

    azimuths[i] = std::atan2(y, x);
    elevations[i] = std::atan2(z, std::sqrt(x * x + y * y));
  }

  return {elevations, azimuths};
}

std::pair<std::array<double, 4>, std::array<double, 4>>
compute_expected_measurements_with_noise(
  const Sophus::SE3d & base_station_pose_in_deck_frame,
  double elevation_noise_mean, double elevation_noise_stddev,
  double azimuth_noise_mean, double azimuth_noise_stddev, std::mt19937 & rng)
{
  auto [elevations, azimuths] = compute_expected_measurements(
    base_station_pose_in_deck_frame);

  std::normal_distribution<double> elevation_noise(elevation_noise_mean,
    elevation_noise_stddev);
  std::normal_distribution<double> azimuth_noise(azimuth_noise_mean,
    azimuth_noise_stddev);

  for (std::size_t i = 0; i < elevations.size(); ++i) {
    elevations[i] += elevation_noise(rng);
    azimuths[i] += azimuth_noise(rng);
  }

  return {elevations, azimuths};
}

double compute_translation_error(
  const Sophus::SE3d & observed_pose,
  const Sophus::SE3d & expected_pose)
{
  const Eigen::Vector3d translation_error =
    observed_pose.translation() - expected_pose.translation();
  return translation_error.norm();
}

double compute_direction_error_radians(
  const Sophus::SE3d & observed_pose,
  const Sophus::SE3d & expected_pose)
{
  const Eigen::Matrix3d R_obs = observed_pose.rotationMatrix();
  const Eigen::Matrix3d R_exp = expected_pose.rotationMatrix();

  const double cos_x = R_obs.col(0).dot(R_exp.col(0));
  const double error_x = std::acos(std::clamp(cos_x, -1.0, 1.0));

  const double cos_z = R_obs.col(2).dot(R_exp.col(2));
  const double error_z = std::acos(std::clamp(cos_z, -1.0, 1.0));

  return std::max(error_x, error_z);
}

Sophus::SE3d create_pose_facing_target(
  const Eigen::Vector3d & position,
  const Eigen::Vector3d & target)
{
  // Direction from position to target (this will be the X-axis)
  Eigen::Vector3d x_axis = (target - position).normalized();

  // Choose an arbitrary up vector (Z-up convention)
  Eigen::Vector3d up(0, 0, 1);

  // Handle the case where x_axis is parallel to up
  if (std::abs(x_axis.dot(up)) > 0.99) {
    up = Eigen::Vector3d(0, 1, 0);
  }

  // Y-axis perpendicular to both X and up
  Eigen::Vector3d y_axis = up.cross(x_axis).normalized();

  // Z-axis perpendicular to both X and Y
  Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();

  // Build rotation matrix
  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;

  return Sophus::SE3d(Sophus::SO3d(rotation), position);
}

Sophus::SE3d add_noise_to_pose(
  const Sophus::SE3d & pose,
  double max_translation_noise,
  double max_rotation_noise, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> trans_dist(-max_translation_noise,
    max_translation_noise);
  std::uniform_real_distribution<double> rot_dist(-max_rotation_noise,
    max_rotation_noise);

  // Add translation noise
  Eigen::Vector3d translation_noise(trans_dist(rng), trans_dist(rng),
    trans_dist(rng));
  Eigen::Vector3d noisy_translation = pose.translation() + translation_noise;

  // Add rotation noise (as small angle perturbation)
  Eigen::Vector3d rotation_noise(rot_dist(rng), rot_dist(rng), rot_dist(rng));
  Sophus::SO3d rotation_perturbation = Sophus::SO3d::exp(rotation_noise);
  Sophus::SO3d noisy_rotation = pose.so3() * rotation_perturbation;

  return Sophus::SE3d(noisy_rotation, noisy_translation);
}

bool solve_sweep_plane_timestamps(
  double t0,
  double rotor_period,
  const Sophus::SE3d & deck_pose_at_t0,
  const Eigen::Vector3d & deck_velocity,
  const Sophus::SE3d & station_pose,
  std::array<double, 8> & sensor_timestamps)
{
  // Create the cost functor for both sweeps (8 residuals, 8 parameters)
  ceres::CostFunction * cost_function =
    new ceres::AutoDiffCostFunction<SweepPlaneTimestampFunctor, 8, 8>(
    new SweepPlaneTimestampFunctor(
      t0, rotor_period, deck_pose_at_t0, deck_velocity, station_pose));

  // Set up the optimization problem
  ceres::Problem problem;
  problem.AddResidualBlock(cost_function, nullptr, sensor_timestamps.data());

  // Configure the solver
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 1000;

  // Solve
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Post-process: wrap all timestamps to be in the same rotor cycle
  // Ceres may find valid plane intersections in different cycles due to
  // the periodic nature of the rotating sweep planes
  if (summary.termination_type == ceres::CONVERGENCE) {
    // For each sweep, ensure all sensor timestamps are in the same cycle
    for (int sweep_idx = 0; sweep_idx < 2; ++sweep_idx) {
      // Compute which cycle each timestamp is in
      std::array<int, 4> cycles;
      std::array<double, 4> wrapped_times;

      for (int sensor_idx = 0; sensor_idx < 4; ++sensor_idx) {
        const int idx = sweep_idx * 4 + sensor_idx;
        const double dt = sensor_timestamps[idx] - t0;
        cycles[sensor_idx] = static_cast<int>(std::floor(dt / rotor_period));
        wrapped_times[sensor_idx] = dt - cycles[sensor_idx] * rotor_period;
      }

      // Use the most common cycle (or first sensor's cycle if all different)
      const int target_cycle = cycles[0];

      // Wrap all sensors to the target cycle
      for (int sensor_idx = 0; sensor_idx < 4; ++sensor_idx) {
        const int idx = sweep_idx * 4 + sensor_idx;
        const int cycle_diff = cycles[sensor_idx] - target_cycle;
        sensor_timestamps[idx] -= cycle_diff * rotor_period;
      }
    }
  }

  // Return true if optimization converged
  return summary.termination_type == ceres::CONVERGENCE;
}

}  // namespace lighthouse_geometry_utils::test
