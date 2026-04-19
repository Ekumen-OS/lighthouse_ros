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
using test::create_pose_facing_target;

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
    create_pose_facing_target({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0});

  const auto [e0, a0] =
    compute_expected_measurements(station_pose, kLighthouseDeckSensorPoses);
  uut->addSample(e0, a0, /*station_id=*/ 0, /*deck_pose_id=*/ 0);

  const auto [e1, a1] =
    compute_expected_measurements(station_pose, kLighthouseDeckSensorPoses);
  uut->addSample(e1, a1, /*station_id=*/ 1, /*deck_pose_id=*/ 1);

  EXPECT_THROW(uut->solve(), std::runtime_error);
}

TEST_F(StationGeometryOptimizationTest, ResetAndSolveWithNoSamplesThrows) {
  const Sophus::SE3d station_pose =
    create_pose_facing_target({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0});

  const auto [e, a] =
    compute_expected_measurements(station_pose, kLighthouseDeckSensorPoses);
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
  static constexpr double kMaxStationRotationOffset = deg2rad(30.0);

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
        station_in_deck, kLighthouseDeckSensorPoses,
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
    {create_pose_facing_target({0.3, 0.2, 1.5}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({-0.3, -0.2, 2.0}, {0.0, 0.0, 0.0})},
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
    {create_pose_facing_target({1.0, 1.0, 3.0}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({-1.0, 1.0, 3.0}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({1.0, -1.0, 3.0}, {0.0, 0.0, 0.0})},
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
    {create_pose_facing_target({1.5, 1.0, 2.5}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({-1.5, 0.5, 3.0}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({0.0, -1.5, 2.8}, {0.0, 0.0, 0.0})},
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
    {create_pose_facing_target({-1.5, 0.0, 1.5}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({-0.5, 1.0, 2.0}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({0.5, -1.0, 1.8}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({1.5, 0.5, 1.6}, {0.0, 0.0, 0.0})},
    /* deck_poses = */
    {Sophus::SE3d{},
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.3, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 0.3, 0.0))},
    /* observations = */
    {{0, 0}, {0, 1}, {1, 1}, {1, 2}, {2, 2}, {2, 3}},
    /* description = */ "FourStationsLine_OneDeckBetweenEachPair"},

    // 7x7 grid: 16 stations (4x4, 2m apart at z=2.5), 64 deck poses
    // (8x8, 1m apart), connected to stations within 3m
    OptimizationScenario{
    /* station_poses = */
    {create_pose_facing_target({0.0, 0.0, 2.5}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({2.0, 0.0, 2.5}, {2.0, 0.0, 0.0}),
      create_pose_facing_target({4.0, 0.0, 2.5}, {4.0, 0.0, 0.0}),
      create_pose_facing_target({6.0, 0.0, 2.5}, {6.0, 0.0, 0.0}),
      create_pose_facing_target({0.0, 2.0, 2.5}, {0.0, 2.0, 0.0}),
      create_pose_facing_target({2.0, 2.0, 2.5}, {2.0, 2.0, 0.0}),
      create_pose_facing_target({4.0, 2.0, 2.5}, {4.0, 2.0, 0.0}),
      create_pose_facing_target({6.0, 2.0, 2.5}, {6.0, 2.0, 0.0}),
      create_pose_facing_target({0.0, 4.0, 2.5}, {0.0, 4.0, 0.0}),
      create_pose_facing_target({2.0, 4.0, 2.5}, {2.0, 4.0, 0.0}),
      create_pose_facing_target({4.0, 4.0, 2.5}, {4.0, 4.0, 0.0}),
      create_pose_facing_target({6.0, 4.0, 2.5}, {6.0, 4.0, 0.0}),
      create_pose_facing_target({0.0, 6.0, 2.5}, {0.0, 6.0, 0.0}),
      create_pose_facing_target({2.0, 6.0, 2.5}, {2.0, 6.0, 0.0}),
      create_pose_facing_target({4.0, 6.0, 2.5}, {4.0, 6.0, 0.0}),
      create_pose_facing_target({6.0, 6.0, 2.5}, {6.0, 6.0, 0.0})},
    /* deck_poses = */
    {Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(1.0, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(2.0, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(3.0, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(4.0, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(5.0, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(6.0, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(7.0, 0.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 1.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(1.0, 1.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(2.0, 1.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(3.0, 1.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(4.0, 1.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(5.0, 1.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(6.0, 1.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(7.0, 1.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 2.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(1.0, 2.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(2.0, 2.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(3.0, 2.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(4.0, 2.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(5.0, 2.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(6.0, 2.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(7.0, 2.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 3.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(1.0, 3.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(2.0, 3.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(3.0, 3.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(4.0, 3.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(5.0, 3.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(6.0, 3.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(7.0, 3.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 4.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(1.0, 4.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(2.0, 4.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(3.0, 4.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(4.0, 4.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(5.0, 4.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(6.0, 4.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(7.0, 4.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 5.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(1.0, 5.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(2.0, 5.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(3.0, 5.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(4.0, 5.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(5.0, 5.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(6.0, 5.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(7.0, 5.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 6.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(1.0, 6.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(2.0, 6.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(3.0, 6.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(4.0, 6.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(5.0, 6.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(6.0, 6.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(7.0, 6.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 7.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(1.0, 7.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(2.0, 7.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(3.0, 7.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(4.0, 7.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(5.0, 7.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(6.0, 7.0, 0.0)),
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(7.0, 7.0, 0.0))},
    /* observations = */
    {{0, 0}, {1, 0}, {1, 1}, {2, 1},                 //
      {3, 1}, {3, 2}, {4, 2}, {5, 2},                //
      {5, 3}, {6, 3}, {7, 3}, {8, 0},                //
      {8, 4}, {9, 0}, {9, 1}, {9, 4},                //
      {9, 5}, {10, 1}, {10, 5}, {11, 1},             //
      {11, 2}, {11, 5}, {11, 6}, {12, 2},            //
      {12, 6}, {13, 2}, {13, 3}, {13, 6},            //
      {13, 7}, {14, 3}, {14, 7}, {15, 3},            //
      {15, 7}, {16, 4}, {17, 4}, {17, 5},            //
      {18, 5}, {19, 5}, {19, 6}, {20, 6},            //
      {21, 6}, {21, 7}, {22, 7}, {23, 7},            //
      {24, 4}, {24, 8}, {25, 4}, {25, 5},            //
      {25, 8}, {25, 9}, {26, 5}, {26, 9},            //
      {27, 5}, {27, 6}, {27, 9}, {27, 10},           //
      {28, 6}, {28, 10}, {29, 6}, {29, 7},           //
      {29, 10}, {29, 11}, {30, 7}, {30, 11},         //
      {31, 7}, {31, 11}, {32, 8}, {33, 8},           //
      {33, 9}, {34, 9}, {35, 9}, {35, 10},           //
      {36, 10}, {37, 10}, {37, 11}, {38, 11},        //
      {39, 11}, {40, 8}, {40, 12}, {41, 8},          //
      {41, 9}, {41, 12}, {41, 13}, {42, 9},          //
      {42, 13}, {43, 9}, {43, 10}, {43, 13},         //
      {43, 14}, {44, 10}, {44, 14}, {45, 10},        //
      {45, 11}, {45, 14}, {45, 15}, {46, 11},        //
      {46, 15}, {47, 11}, {47, 15}, {48, 12},        //
      {49, 12}, {49, 13}, {50, 13}, {51, 13},        //
      {51, 14}, {52, 14}, {53, 14}, {53, 15},        //
      {54, 15}, {55, 15}, {56, 12}, {57, 12},        //
      {57, 13}, {58, 13}, {59, 13}, {59, 14},        //
      {60, 14}, {61, 14}, {61, 15}, {62, 15},        //
      {63, 15}},
    /* description = */ "Grid7x7_16Stations_64Decks"}),
  [](const ::testing::TestParamInfo<OptimizationScenario> & info) {
    return info.param.description;
  });

}  // namespace lighthouse_geometry_utils

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
