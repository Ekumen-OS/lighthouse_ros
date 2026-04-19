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

#include "lighthouse_geometry_utils/station_geometry_optimization.hpp"

#include <ceres/ceres.h>

#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "lighthouse_geometry_utils/ceres_helpers.hpp"
#include "lighthouse_geometry_utils/utils.hpp"
#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils
{

StationGeometryOptimization::StationGeometryOptimization()
: sensor_poses_{
    kLighthouseDeckSensorPoses[0],
    kLighthouseDeckSensorPoses[1],
    kLighthouseDeckSensorPoses[2],
    kLighthouseDeckSensorPoses[3],
} {}

void StationGeometryOptimization::addSample(
  const std::array<double, 4> & elevations,
  const std::array<double, 4> & azimuths, StationId station_id,
  DeckPoseId deck_pose_id)
{
  deck_pose_data_[deck_pose_id].station_data.push_back(
    {elevations, azimuths, station_id});
  initializer_.addSample(elevations, azimuths, station_id, deck_pose_id);

  // Track unique station IDs for validation in solve()
  if (std::find(station_ids_.begin(), station_ids_.end(), station_id) ==
    station_ids_.end())
  {
    station_ids_.push_back(station_id);
  }
}

void StationGeometryOptimization::reset()
{
  deck_pose_data_.clear();
  station_ids_.clear();
  initializer_.reset();
}

StationPoseEstimates StationGeometryOptimization::solve()
{
  // Use the initializer to compute initial pose estimates for all stations and
  // decks.
  const NetworkNodePoses initial_poses =
    initializer_.solve();

  std::unordered_map<StationId, std::size_t> station_id_to_index;

  // Initialize station poses from the initializer result.
  std::vector<Sophus::SE3d> station_pose_estimations;

  for (std::size_t i = 0; i < station_ids_.size(); ++i) {
    station_pose_estimations.push_back(
      initial_poses.station_poses.at(station_ids_[i]));
    station_id_to_index[station_ids_[i]] = i;
  }

  // Create deck poses vector
  std::vector<Sophus::SE3d> deck_poses_estimation;

  // Map deck IDs to indices in the optimization vector
  std::unordered_map<DeckPoseId, std::size_t> deck_id_to_index;

  // Add all real deck poses from the initializer result.
  for (const auto & entry : deck_pose_data_) {
    const auto deck_pose_id = entry.first;
    deck_id_to_index[deck_pose_id] = deck_poses_estimation.size();
    deck_poses_estimation.push_back(initial_poses.deck_poses.at(deck_pose_id));
  }

  // now we have initial estimates for all deck poses and all station poses
  // we create the ceres problem and add residuals for all samples
  ceres::Problem problem;

  // Create parameterization for SE3 optimization (Ceres will take ownership)
  ceres::LocalParameterization * se3_parameterization =
    createSE3Parameterization();

  // Step 1: Add all parameter blocks with SE3 parameterization
  // This must be done BEFORE adding residual blocks

  // Add all deck pose parameter blocks (including virtual deck at index 0)
  for (auto & deck_pose : deck_poses_estimation) {
    problem.AddParameterBlock(deck_pose.data(), 7, se3_parameterization);
  }

  // Add all station pose parameter blocks
  for (auto & station_pose : station_pose_estimations) {
    problem.AddParameterBlock(station_pose.data(), 7, se3_parameterization);
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

  // Fix the first deck to establish the reference frame
  problem.SetParameterBlockConstant(deck_poses_estimation[0].data());

  // Step 2: Add bearing vector residual blocks
  for (const auto & entry : deck_pose_data_) {
    const auto deck_pose_id = entry.first;
    const auto & location_data = entry.second;

    const auto deck_it = deck_id_to_index.find(deck_pose_id);
    const auto deck_index = deck_it->second;
    double * deck_pose_data_ = deck_poses_estimation[deck_index].data();

    for (const auto & sample : location_data.station_data) {
      const auto station_id = sample.station_id;
      const auto it = station_id_to_index.find(station_id);
      const auto station_index = it->second;
      double * station_pose_data =
        station_pose_estimations[station_index].data();

      problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<BearingVectorErrorFunctor, 12, 7, 7,
        8>(
          new BearingVectorErrorFunctor(
            sample.elevations, sample.azimuths, sensor_poses_)),
        new ceres::HuberLoss(kHuberDeltaStations), deck_pose_data_,
        station_pose_data, station_biases[station_index].data());
    }
  }

  // Configure and run the solver
  // Note: The virtual deck (index 0) is already set as constant to establish
  // the reference frame, so we don't need to fix any station poses
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 100;
  options.use_nonmonotonic_steps = true;
  options.max_consecutive_nonmonotonic_steps = 10;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Compute autocovariance for each station pose
  std::vector<AutoCovDiagonal> station_autocov_diagonals(station_ids_.size());
  ceres::Covariance::Options cov_options;
  ceres::Covariance covariance(cov_options);

  std::vector<std::pair<const double *, const double *>> cov_blocks;
  for (std::size_t i = 0; i < station_pose_estimations.size(); ++i) {
    cov_blocks.emplace_back(
      station_pose_estimations[i].data(), station_pose_estimations[i].data());
  }

  if (covariance.Compute(cov_blocks, &problem)) {
    for (std::size_t i = 0; i < station_pose_estimations.size(); ++i) {
      double cov_matrix[6 * 6];
      covariance.GetCovarianceBlockInTangentSpace(
        station_pose_estimations[i].data(), station_pose_estimations[i].data(),
        cov_matrix);
      station_autocov_diagonals[i] = extractAutoCovDiagonal(cov_matrix);
    }
  }

  return StationPoseEstimates{
    station_pose_estimations, station_ids_, station_autocov_diagonals};
}

}  // namespace lighthouse_geometry_utils
