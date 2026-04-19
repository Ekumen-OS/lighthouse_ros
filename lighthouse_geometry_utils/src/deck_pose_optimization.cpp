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

#include "lighthouse_geometry_utils/deck_pose_optimization.hpp"

#include <ceres/ceres.h>

#include <algorithm>
#include <stdexcept>
#include <unordered_map>

#include "lighthouse_geometry_utils/ceres_helpers.hpp"
#include "lighthouse_geometry_utils/station_pose_pnp_solver.hpp"
#include "lighthouse_geometry_utils/utils.hpp"
#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils
{

DeckPoseOptimization::DeckPoseOptimization(
  const std::vector<Sophus::SE3d> & station_poses,
  const std::vector<StationId> & station_ids)
: station_poses_(station_poses),
  station_ids_(station_ids),
  sensor_poses_{
    kLighthouseDeckSensorPoses[0],
    kLighthouseDeckSensorPoses[1],
    kLighthouseDeckSensorPoses[2],
    kLighthouseDeckSensorPoses[3],
  } {}

std::tuple<Sophus::SE3d, AutoCovDiagonal> DeckPoseOptimization::solve(
  const std::vector<Sample> & samples) const
{
  // Check for empty station poses
  if (station_poses_.empty()) {
    throw std::invalid_argument(
            "DeckPoseOptimization::solve: No station poses provided");
  }

  // Check for empty samples
  if (samples.empty()) {
    throw std::invalid_argument(
            "DeckPoseOptimization::solve: No samples provided");
  }

  // Build a map from station ID to index in station_poses_
  std::unordered_map<StationId, std::size_t> station_id_to_index;
  for (std::size_t i = 0; i < station_ids_.size(); ++i) {
    station_id_to_index[station_ids_[i]] = i;
  }

  // Filter samples to only include those with known station IDs
  std::vector<Sample> valid_samples;
  for (const auto & sample : samples) {
    if (station_id_to_index.find(sample.station_id) !=
      station_id_to_index.end())
    {
      valid_samples.push_back(sample);
    }
  }

  // Check if all samples were discarded
  if (valid_samples.empty()) {
    throw std::invalid_argument(
            "DeckPoseOptimization::solve: All samples have unknown station IDs");
  }

  // Compute initial deck pose estimate using PnP from the first valid sample
  StationPosePnPSolver pnp_solver;
  Sophus::SE3d deck_pose_estimation;
  {
    const auto & first_sample = valid_samples.front();
    const auto it = station_id_to_index.find(first_sample.station_id);
    const Sophus::SE3d & station_pose = station_poses_[it->second];
    // PnP gives station-in-deck; we need deck-in-world
    const Sophus::SE3d station_in_deck =
      pnp_solver.solve(first_sample.elevations, first_sample.azimuths);
    deck_pose_estimation = station_pose * station_in_deck.inverse();
  }

  // Make mutable copies of station poses for Ceres parameter blocks
  std::vector<Sophus::SE3d> station_poses_mutable = station_poses_;

  ceres::Problem problem;

  ceres::LocalParameterization * se3_parameterization =
    createSE3Parameterization();

  // Add deck pose parameter block
  problem.AddParameterBlock(
    deck_pose_estimation.data(), 7, se3_parameterization);

  // Add station pose parameter blocks as constants
  for (auto & station_pose : station_poses_mutable) {
    problem.AddParameterBlock(station_pose.data(), 7, se3_parameterization);
    problem.SetParameterBlockConstant(station_pose.data());
  }

  // Create bias parameter blocks (8 per station: 4 elevation + 4 azimuth
  // offsets), initialized to zero
  std::vector<std::array<double, 8>> station_biases(station_ids_.size());
  for (auto & bias : station_biases) {
    bias.fill(0.0);
  }

  // Add bias parameter blocks and regularization residuals
  for (std::size_t i = 0; i < station_biases.size(); ++i) {
    problem.AddParameterBlock(station_biases[i].data(), 8);
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<BiasRegularizationFunctor, 8, 8>(
        new BiasRegularizationFunctor()),
      new ceres::HuberLoss(kHuberDeltaBiasRegularization),
      station_biases[i].data());
  }

  // Add bearing vector residual blocks (using valid_samples)
  for (const auto & sample : valid_samples) {
    const auto it = station_id_to_index.find(sample.station_id);
    const auto station_index = it->second;

    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<BearingVectorErrorFunctor, 12, 7, 7, 8>(
        new BearingVectorErrorFunctor(
          sample.elevations, sample.azimuths, sensor_poses_)),
      new ceres::HuberLoss(kHuberDeltaStations),
      deck_pose_estimation.data(),
      station_poses_mutable[station_index].data(),
      station_biases[station_index].data());
  }

  // Configure and run the solver
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 100;
  options.use_nonmonotonic_steps = true;
  options.max_consecutive_nonmonotonic_steps = 10;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Compute covariance of the deck pose
  AutoCovDiagonal auto_cov{};
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);

  std::vector<std::pair<const double *, const double *>> cov_blocks;
  cov_blocks.emplace_back(
    deck_pose_estimation.data(), deck_pose_estimation.data());

  if (covariance.Compute(cov_blocks, &problem)) {
    double cov_matrix[6 * 6];
    covariance.GetCovarianceBlockInTangentSpace(
      deck_pose_estimation.data(), deck_pose_estimation.data(), cov_matrix);
    auto_cov = extractAutoCovDiagonal(cov_matrix);
  }

  return {deck_pose_estimation, auto_cov};
}

}  // namespace lighthouse_geometry_utils
