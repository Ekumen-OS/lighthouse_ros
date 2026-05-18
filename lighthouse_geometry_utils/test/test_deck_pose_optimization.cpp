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

#include "lighthouse_geometry_utils/deck_pose_optimization.hpp"
#include "lighthouse_geometry_utils/utils.hpp"
#include "test_helpers.hpp"
#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils
{
using test::add_noise_to_pose;
using test::compute_direction_error_radians;
using test::compute_expected_measurements_with_noise;
using test::compute_translation_error;
using test::create_pose_facing_target;

constexpr double deg2rad(double degrees) {return degrees * M_PI / 180.0;}

struct DeckPoseScenario
{
  /// Ground truth deck pose in world frame.
  Sophus::SE3d deck_pose;
  /// Ground truth station poses in world frame.
  std::vector<Sophus::SE3d> station_poses;
  std::string description;
};

class DeckPoseOptimizationPoseTest
  : public ::testing::TestWithParam<DeckPoseScenario>
{
protected:
  static constexpr int kSolverTestPasses = 50;

  static constexpr double kMeasurementNoiseMean = deg2rad(0.0);
  static constexpr double kMeasurementNoiseStddev = deg2rad(0.0001);

  static constexpr double kMaxDeckPositionOffset = 0.5;
  static constexpr double kMaxDeckRotationOffset = deg2rad(30.0);

  static constexpr double kExpectedTranslationError = 0.05;
  static constexpr double kExpectedDirectionError = deg2rad(5.0);
};

TEST_P(DeckPoseOptimizationPoseTest, RecoveredDeckPoseMatchesGroundTruth) {
  const auto & scenario = GetParam();
  std::mt19937 rng(42);

  const std::size_t num_stations = scenario.station_poses.size();

  // Build station IDs
  std::vector<StationId> station_ids(num_stations);
  for (std::size_t i = 0; i < num_stations; ++i) {
    station_ids[i] = i;
  }

  for (int pass = 0; pass < kSolverTestPasses; ++pass) {
    // Apply random offset to the deck pose for per-pass variation
    const Sophus::SE3d deck_pose = add_noise_to_pose(
      scenario.deck_pose, kMaxDeckPositionOffset, kMaxDeckRotationOffset, rng);

    // Station poses are known exactly — no noise on them
    DeckPoseOptimization optimizer(scenario.station_poses, station_ids);

    // Build measurement samples from all stations at this deck pose
    std::vector<DeckPoseOptimization::Sample> samples;
    for (std::size_t station_id = 0; station_id < num_stations; ++station_id) {
      const Sophus::SE3d station_in_deck =
        deck_pose.inverse() * scenario.station_poses[station_id];
      const auto [elevations, azimuths] =
        compute_expected_measurements_with_noise(
        station_in_deck,
        kMeasurementNoiseMean, kMeasurementNoiseStddev,
        kMeasurementNoiseMean, kMeasurementNoiseStddev, rng);
      samples.push_back({elevations, azimuths, station_id});
    }

    const auto [recovered, auto_cov] = optimizer.solve(samples);
    (void)auto_cov;

    ASSERT_NEAR(
      compute_translation_error(recovered, deck_pose), 0.0,
      kExpectedTranslationError)
      << scenario.description << " pass " << pass << " translation";
    ASSERT_NEAR(
      compute_direction_error_radians(recovered, deck_pose), 0.0,
      kExpectedDirectionError)
      << scenario.description << " pass " << pass << " direction";
  }
}

INSTANTIATE_TEST_SUITE_P(
  Scenarios, DeckPoseOptimizationPoseTest,
  ::testing::Values(
    // Single station above deck
    DeckPoseScenario{
    /* deck_pose = */
    Sophus::SE3d{},
    /* station_poses = */
    {create_pose_facing_target({0.0, 0.0, 2.0}, {0.0, 0.0, 0.0})},
    /* description = */ "SingleStation"},

    // Two stations
    DeckPoseScenario{
    /* deck_pose = */
    Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.5, 0.3, 0.0)),
    /* station_poses = */
    {create_pose_facing_target({0.3, 0.2, 1.5}, {0.5, 0.3, 0.0}),
      create_pose_facing_target({-0.3, -0.2, 2.0}, {0.5, 0.3, 0.0})},
    /* description = */ "TwoStations"},

    // Three stations in a triangle
    DeckPoseScenario{
    /* deck_pose = */
    Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(1.0, 1.0, 0.0)),
    /* station_poses = */
    {create_pose_facing_target({1.0, 1.0, 3.0}, {1.0, 1.0, 0.0}),
      create_pose_facing_target({-1.0, 1.0, 3.0}, {1.0, 1.0, 0.0}),
      create_pose_facing_target({1.0, -1.0, 3.0}, {1.0, 1.0, 0.0})},
    /* description = */ "ThreeStationsTriangle"},

    // Four stations surrounding the deck
    DeckPoseScenario{
    /* deck_pose = */
    Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.0, 0.0, 0.0)),
    /* station_poses = */
    {create_pose_facing_target({-1.5, 0.0, 1.5}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({-0.5, 1.0, 2.0}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({0.5, -1.0, 1.8}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({1.5, 0.5, 1.6}, {0.0, 0.0, 0.0})},
    /* description = */ "FourStationsSurrounding"}),
  [](const ::testing::TestParamInfo<DeckPoseScenario> & info) {
    return info.param.description;
  });

// Error handling tests
TEST(DeckPoseOptimizationErrorTest, ThrowsOnEmptyStationPoses) {
  // Empty station poses and IDs
  std::vector<Sophus::SE3d> station_poses;
  std::vector<StationId> station_ids;

  DeckPoseOptimization optimizer(station_poses, station_ids);

  // Create a valid sample
  DeckPoseOptimization::Sample sample;
  sample.elevations = {0.1, 0.2, 0.3, 0.4};
  sample.azimuths = {0.5, 0.6, 0.7, 0.8};
  sample.station_id = 0;

  std::vector<DeckPoseOptimization::Sample> samples = {sample};

  EXPECT_THROW(
    {
      try {
        optimizer.solve(samples);
      } catch (const std::invalid_argument & e) {
        EXPECT_STREQ("DeckPoseOptimization::solve: No station poses provided", e.what());
        throw;
      }
    },
    std::invalid_argument);
}

TEST(DeckPoseOptimizationErrorTest, ThrowsOnEmptySamples) {
  // Create a valid station pose
  Sophus::SE3d station_pose = create_pose_facing_target({0.0, 0.0, 2.0}, {0.0, 0.0, 0.0});
  std::vector<Sophus::SE3d> station_poses = {station_pose};
  std::vector<StationId> station_ids = {0};

  DeckPoseOptimization optimizer(station_poses, station_ids);

  // Empty samples
  std::vector<DeckPoseOptimization::Sample> samples;

  EXPECT_THROW(
    {
      try {
        optimizer.solve(samples);
      } catch (const std::invalid_argument & e) {
        EXPECT_STREQ("DeckPoseOptimization::solve: No samples provided", e.what());
        throw;
      }
    },
    std::invalid_argument);
}

TEST(DeckPoseOptimizationErrorTest, ThrowsOnAllUnknownStationIds) {
  // Create a valid station pose with ID 0
  Sophus::SE3d station_pose = create_pose_facing_target({0.0, 0.0, 2.0}, {0.0, 0.0, 0.0});
  std::vector<Sophus::SE3d> station_poses = {station_pose};
  std::vector<StationId> station_ids = {0};

  DeckPoseOptimization optimizer(station_poses, station_ids);

  // Create samples with unknown station IDs (1 and 2, but optimizer only knows about 0)
  DeckPoseOptimization::Sample sample1;
  sample1.elevations = {0.1, 0.2, 0.3, 0.4};
  sample1.azimuths = {0.5, 0.6, 0.7, 0.8};
  sample1.station_id = 1;

  DeckPoseOptimization::Sample sample2;
  sample2.elevations = {0.2, 0.3, 0.4, 0.5};
  sample2.azimuths = {0.6, 0.7, 0.8, 0.9};
  sample2.station_id = 2;

  std::vector<DeckPoseOptimization::Sample> samples = {sample1, sample2};

  EXPECT_THROW(
    {
      try {
        optimizer.solve(samples);
      } catch (const std::invalid_argument & e) {
        EXPECT_STREQ(
          "DeckPoseOptimization::solve: All samples have unknown station IDs",
          e.what());
        throw;
      }
    },
    std::invalid_argument);
}

TEST(DeckPoseOptimizationErrorTest, HandlesPartiallyUnknownStationIds) {
  // Create a valid station pose with ID 0
  Sophus::SE3d station_pose = create_pose_facing_target({0.0, 0.0, 2.0}, {0.0, 0.0, 0.0});
  std::vector<Sophus::SE3d> station_poses = {station_pose};
  std::vector<StationId> station_ids = {0};

  DeckPoseOptimization optimizer(station_poses, station_ids);

  std::mt19937 rng(42);
  const Sophus::SE3d deck_pose(Sophus::SO3d{}, Eigen::Vector3d(0.0, 0.0, 0.0));
  const Sophus::SE3d station_in_deck = deck_pose.inverse() * station_pose;

  // Create one valid sample (station ID 0) and one invalid (station ID 999)
  const auto [elevations0, azimuths0] =
    compute_expected_measurements_with_noise(
    station_in_deck, 0.0, 0.0, 0.0, 0.0, rng);

  DeckPoseOptimization::Sample valid_sample;
  valid_sample.elevations = elevations0;
  valid_sample.azimuths = azimuths0;
  valid_sample.station_id = 0;

  DeckPoseOptimization::Sample invalid_sample;
  invalid_sample.elevations = {0.2, 0.3, 0.4, 0.5};
  invalid_sample.azimuths = {0.6, 0.7, 0.8, 0.9};
  invalid_sample.station_id = 999;

  std::vector<DeckPoseOptimization::Sample> samples = {invalid_sample, valid_sample};

  // Should succeed by discarding the invalid sample and using the valid one
  const auto [recovered, auto_cov] = optimizer.solve(samples);
  (void)auto_cov;
  // Verify the result is reasonable (close to the deck pose)
  EXPECT_NEAR(
    compute_translation_error(recovered, deck_pose), 0.0, 0.1);
}

}  // namespace lighthouse_geometry_utils

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
