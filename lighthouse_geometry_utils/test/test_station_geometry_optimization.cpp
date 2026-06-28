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

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <random>

#include "lighthouse_geometry_utils/station_geometry_optimization.hpp"
#include "lighthouse_geometry_utils/utils.hpp"
#include "test_helpers.hpp"
#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils
{
using test::add_noise_to_pose;
using test::compute_direction_error_radians;
using test::compute_expected_measurements;
using test::compute_expected_measurements_with_noise;
using test::compute_translation_error;
using test::create_upright_station_pose;

constexpr double deg2rad(double degrees) {return degrees * M_PI / 180.0;}

class StationGeometryOptimizationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    uut = std::make_unique<StationGeometryOptimization>();
  }

  std::unique_ptr<StationGeometryOptimization> uut;
};

TEST_F(StationGeometryOptimizationTest, SolveWithNoSamplesThrows) {
  EXPECT_THROW(uut->solve(), std::runtime_error);
}

TEST_F(StationGeometryOptimizationTest, DisconnectedGraphThrows) {
  // deck 0 -- station 0   and   deck 1 -- station 1: two isolated subgraphs.
  const Sophus::SE3d station_pose =
    create_upright_station_pose({3.0, 0.0, 1.5}, {0.0, 0.0, 0.0});

  const auto [e0, a0] =
    compute_expected_measurements(station_pose);
  uut->addSample(e0, a0, /*station_id=*/ 0, /*deck_pose_id=*/ 0);

  const auto [e1, a1] =
    compute_expected_measurements(station_pose);
  uut->addSample(e1, a1, /*station_id=*/ 1, /*deck_pose_id=*/ 1);

  EXPECT_THROW(uut->solve(), std::runtime_error);
}

TEST_F(StationGeometryOptimizationTest, ResetAndSolveWithNoSamplesThrows) {
  const Sophus::SE3d station_pose =
    create_upright_station_pose({3.0, 0.0, 1.5}, {0.0, 0.0, 0.0});

  const auto [e, a] =
    compute_expected_measurements(station_pose);
  uut->addSample(e, a, /*station_id=*/ 0, /*deck_pose_id=*/ 0);

  uut->reset();

  EXPECT_THROW(uut->solve(), std::runtime_error);
}

struct OptimizationScenario
{
  /// Ground truth station poses in world frame.
  std::vector<Sophus::SE3d> station_poses;
  /// Ground truth deck poses in world frame.
  std::vector<Sophus::SE3d> deck_poses;
  /// Observations as (deck_id, station_id) pairs. The graph must be connected.
  std::vector<std::pair<std::size_t, std::size_t>> observations;
  std::string description;
};

class StationGeometryOptimizationPoseTest
  : public ::testing::TestWithParam<OptimizationScenario>
{
protected:
  static constexpr int kSolverTestPasses = 50;

  static constexpr double kMeasurementNoiseMean = deg2rad(0.0);
  static constexpr double kMeasurementNoiseStddev = deg2rad(0.0001);

  static constexpr double kMaxStationPositionOffset = 0.5;
  static constexpr double kMaxStationRotationOffset = deg2rad(25.0);

  static constexpr double kExpectedTranslationError = 0.05;
  static constexpr double kExpectedDirectionError = deg2rad(5.0);
};

TEST_P(
  StationGeometryOptimizationPoseTest,
  RelativeStationPosesMatchGroundTruth) {
  const auto & scenario = GetParam();
  std::mt19937 rng(42);

  for (int pass = 0; pass < kSolverTestPasses; ++pass) {
    // Apply random offsets to ground truth poses to create per-pass variation.
    std::vector<Sophus::SE3d> station_poses;
    for (const auto & pose : scenario.station_poses) {
      station_poses.push_back(
        add_noise_to_pose(
          pose, kMaxStationPositionOffset, kMaxStationRotationOffset, rng));
    }
    std::vector<Sophus::SE3d> deck_poses;
    for (const auto & pose : scenario.deck_poses) {
      deck_poses.push_back(
        add_noise_to_pose(
          pose, kMaxStationPositionOffset,
          kMaxStationRotationOffset, rng));
    }

    // Feed noisy measurements to a fresh optimizer instance.
    StationGeometryOptimization optimizer;
    for (const auto &[deck_id, station_id] : scenario.observations) {
      const Sophus::SE3d station_in_deck =
        deck_poses[deck_id].inverse() * station_poses[station_id];
      const auto [elevations, azimuths] =
        compute_expected_measurements_with_noise(
        station_in_deck,
        kMeasurementNoiseMean, kMeasurementNoiseStddev,
        kMeasurementNoiseMean, kMeasurementNoiseStddev, rng);
      optimizer.addSample(elevations, azimuths, station_id, deck_id);
    }

    StationPoseEstimates result;
    ASSERT_NO_THROW(result = optimizer.solve())
      << scenario.description << " pass " << pass;

    // Build a map from station_id -> recovered pose for easy lookup.
    std::map<std::size_t, Sophus::SE3d> result_by_id;
    for (std::size_t i = 0; i < result.station_ids.size(); ++i) {
      result_by_id[result.station_ids[i]] = result.station_poses[i];
    }

    // Compare relative poses of each station against station 0.
    const std::size_t ref_station_id = scenario.observations.front().second;
    for (std::size_t i = 0; i < station_poses.size(); ++i) {
      if (i == ref_station_id) {
        continue;
      }

      const Sophus::SE3d gt_relative =
        station_poses[ref_station_id].inverse() * station_poses[i];
      const Sophus::SE3d recovered_relative =
        result_by_id.at(ref_station_id).inverse() * result_by_id.at(i);

      ASSERT_NEAR(
        compute_translation_error(recovered_relative, gt_relative),
        0.0, kExpectedTranslationError)
        << scenario.description << " pass " << pass << " station " << i
        << " translation";
      ASSERT_NEAR(
        compute_direction_error_radians(recovered_relative, gt_relative), 0.0,
        kExpectedDirectionError)
        << scenario.description << " pass " << pass << " station " << i
        << " direction";
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  Scenarios, StationGeometryOptimizationPoseTest,
  ::testing::Values(
    // Two stations observed by two decks each
    OptimizationScenario{
    /* station_poses = */
    {create_upright_station_pose({4.0, 2.0, 1.5}, {0.0, 0.0, 0.0}),
      create_upright_station_pose({-3.5, -3.0, 2.0}, {0.0, 0.0, 0.0})},
    /* deck_poses = */
    {Sophus::SE3d{},
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.4, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 0.4, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(-0.4, 0.0, 0.0))},
    /* observations = */
    {{0, 0}, {0, 1}, {1, 0}, {1, 1}, {2, 0}, {2, 1}, {3, 0}, {3, 1}},
    /* description = */ "TwoStationsFourDecks"},

    // Three stations A(0) B(1) C(2): two decks between A-B, three between
    // B-C
    OptimizationScenario{
    /* station_poses = */
    {create_upright_station_pose({6.0, 4.0, 3.0}, {0.0, 0.0, 0.0}),
      create_upright_station_pose({-5.5, 5.0, 3.0}, {0.0, 0.0, 0.0}),
      create_upright_station_pose({5.0, -6.0, 3.0}, {0.0, 0.0, 0.0})},
    /* deck_poses = */
    {Sophus::SE3d{},
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.3, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 0.3, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(-0.3, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, -0.3, 0.0))},
    /* observations = */
    {{0, 0},
      {0, 1},
      {1, 0},
      {1, 1},
      {2, 1},
      {2, 2},
      {3, 1},
      {3, 2},
      {4, 1},
      {4, 2}},
    /* description = */ "ThreeStationsTwoBetweenAB_ThreeBetweenBC"},

    // Three stations with full ring coverage: one deck A-B, two B-C, three
    // C-A
    OptimizationScenario{
    /* station_poses = */
    {create_upright_station_pose({5.5, 4.0, 2.5}, {0.0, 0.0, 0.0}),
      create_upright_station_pose({-5.5, 3.0, 3.0}, {0.0, 0.0, 0.0}),
      create_upright_station_pose({4.0, -5.5, 2.8}, {0.0, 0.0, 0.0})},
    /* deck_poses = */
    {Sophus::SE3d{},
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.3, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 0.3, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(-0.3, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, -0.3, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.2, 0.2, 0.0))},
    /* observations = */
    {{0, 0},
      {0, 1},
      {1, 1},
      {1, 2},
      {2, 1},
      {2, 2},
      {3, 2},
      {3, 0},
      {4, 2},
      {4, 0},
      {5, 2},
      {5, 0}},
    /* description = */
    "ThreeStationsRing_OneBetweenAB_TwoBetweenBC_ThreeBetweenCA"},

    // Four stations in a line, connected by one deck between each pair
    OptimizationScenario{
    /* station_poses = */
    {create_upright_station_pose({-4.5, 0.0, 1.5}, {0.0, 0.0, 0.0}),
      create_upright_station_pose({-2.0, 4.0, 2.0}, {0.0, 0.0, 0.0}),
      create_upright_station_pose({2.5, -4.0, 1.8}, {0.0, 0.0, 0.0}),
      create_upright_station_pose({5.0, 2.5, 1.6}, {0.0, 0.0, 0.0})},
    /* deck_poses = */
    {Sophus::SE3d{},
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.3, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 0.3, 0.0))},
    /* observations = */
    {{0, 0}, {0, 1}, {1, 1}, {1, 2}, {2, 2}, {2, 3}},
    /* description = */ "FourStationsLine_OneDeckBetweenEachPair"}),
  [](const ::testing::TestParamInfo<OptimizationScenario> & info) {
    return info.param.description;
  });

}  // namespace lighthouse_geometry_utils

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
