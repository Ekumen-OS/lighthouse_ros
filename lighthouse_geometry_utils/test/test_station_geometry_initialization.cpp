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

#include "lighthouse_geometry_utils/station_geometry_initialization.hpp"
#include "lighthouse_geometry_utils/utils.hpp"
#include "test_helpers.hpp"

#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils
{
using test::compute_direction_error_radians;
using test::compute_expected_measurements;
using test::compute_translation_error;
using test::create_pose_facing_target;

class StationGeometryInitializationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    uut = std::make_unique<StationGeometryInitialization>();
  }

  std::unique_ptr<StationGeometryInitialization> uut;
};

// Helper to generate a valid measurement from a known station pose
std::pair<std::array<double, 4>, std::array<double, 4>>
make_measurement(const Sophus::SE3d & station_pose_in_deck)
{
  return compute_expected_measurements(station_pose_in_deck);
}

// ---------- Error case tests ----------

TEST_F(StationGeometryInitializationTest, SolveWithNoSamplesThrows) {
  EXPECT_THROW(uut->solve(), std::runtime_error);
}

TEST_F(StationGeometryInitializationTest, DisconnectedGraphThrows) {
  // Add two isolated deck-station pairs with no shared node between them.
  // deck 0 -- station 0
  // deck 1 -- station 1  (no edge connecting these two subgraphs)
  const Sophus::SE3d station_pose =
    create_pose_facing_target({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0});

  const auto [elevations0, azimuths0] = make_measurement(station_pose);
  uut->addSample(elevations0, azimuths0, /*station_id=*/ 0, /*deck_pose_id=*/ 0);

  const auto [elevations1, azimuths1] = make_measurement(station_pose);
  uut->addSample(elevations1, azimuths1, /*station_id=*/ 1, /*deck_pose_id=*/ 1);

  EXPECT_THROW(uut->solve(), std::runtime_error);
}

// ---------- Happy path tests ----------

TEST_F(StationGeometryInitializationTest, SingleSampleReturnsResult) {
  const Sophus::SE3d station_pose =
    create_pose_facing_target({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0});

  const auto [elevations, azimuths] = make_measurement(station_pose);
  uut->addSample(elevations, azimuths, /*station_id=*/ 0, /*deck_pose_id=*/ 0);

  NetworkNodePoses result;
  ASSERT_NO_THROW(result = uut->solve());

  EXPECT_EQ(result.station_poses.size(), 1u);
  EXPECT_EQ(result.deck_poses.size(), 1u);
  EXPECT_TRUE(result.station_poses.count(0));
  EXPECT_TRUE(result.deck_poses.count(0));
}

TEST_F(StationGeometryInitializationTest, RootDeckHasIdentityPose) {
  const Sophus::SE3d station_pose =
    create_pose_facing_target({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0});

  const auto [elevations, azimuths] = make_measurement(station_pose);
  uut->addSample(elevations, azimuths, /*station_id=*/ 0, /*deck_pose_id=*/ 0);

  const auto result = uut->solve();

  // The first deck should be the root with identity pose
  const Sophus::SE3d & root_pose = result.deck_poses.at(0);
  EXPECT_NEAR(root_pose.translation().norm(), 0.0, 1e-9);
  EXPECT_NEAR(root_pose.so3().log().norm(), 0.0, 1e-9);
}

TEST_F(StationGeometryInitializationTest, TwoDecksOneStationConnected) {
  // Both decks observe the same station: deck0 -- station0 -- deck1
  const Sophus::SE3d station_pose =
    create_pose_facing_target({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0});

  const auto [elevations0, azimuths0] = make_measurement(station_pose);
  uut->addSample(elevations0, azimuths0, /*station_id=*/ 0, /*deck_pose_id=*/ 0);

  const auto [elevations1, azimuths1] = make_measurement(station_pose);
  uut->addSample(elevations1, azimuths1, /*station_id=*/ 0, /*deck_pose_id=*/ 1);

  NetworkNodePoses result;
  ASSERT_NO_THROW(result = uut->solve());

  EXPECT_EQ(result.station_poses.size(), 1u);
  EXPECT_EQ(result.deck_poses.size(), 2u);
}

TEST_F(StationGeometryInitializationTest, AllKnownIdsAppearInResult) {
  const Sophus::SE3d station_pose =
    create_pose_facing_target({0.3, 0.0, 1.5}, {0.0, 0.0, 0.0});

  // deck 0 sees station 0 and station 1; deck 1 sees station 1
  const Sophus::SE3d station_pose2 =
    create_pose_facing_target({-0.3, 0.0, 1.5}, {0.0, 0.0, 0.0});

  const auto [e0, a0] = make_measurement(station_pose);
  uut->addSample(e0, a0, /*station_id=*/ 0, /*deck_pose_id=*/ 0);

  const auto [e1, a1] = make_measurement(station_pose2);
  uut->addSample(e1, a1, /*station_id=*/ 1, /*deck_pose_id=*/ 0);

  const auto [e2, a2] = make_measurement(station_pose2);
  uut->addSample(e2, a2, /*station_id=*/ 1, /*deck_pose_id=*/ 1);

  NetworkNodePoses result;
  ASSERT_NO_THROW(result = uut->solve());

  EXPECT_EQ(result.station_poses.size(), 2u);
  EXPECT_EQ(result.deck_poses.size(), 2u);
  EXPECT_TRUE(result.station_poses.count(0));
  EXPECT_TRUE(result.station_poses.count(1));
  EXPECT_TRUE(result.deck_poses.count(0));
  EXPECT_TRUE(result.deck_poses.count(1));
}

TEST_F(StationGeometryInitializationTest, ResetClearsState) {
  const Sophus::SE3d station_pose =
    create_pose_facing_target({0.0, 0.0, 1.5}, {0.0, 0.0, 0.0});

  // Add a sample and verify solve succeeds.
  const auto [elevations, azimuths] = make_measurement(station_pose);
  uut->addSample(elevations, azimuths, /*station_id=*/ 0, /*deck_pose_id=*/ 0);
  ASSERT_NO_THROW(uut->solve());

  // After reset, solve should throw because there are no samples.
  uut->reset();
  EXPECT_THROW(uut->solve(), std::runtime_error);
}

TEST_F(StationGeometryInitializationTest, ResetAllowsReuse) {
  const Sophus::SE3d station_pose_a =
    create_pose_facing_target({0.3, 0.2, 1.5}, {0.0, 0.0, 0.0});
  const Sophus::SE3d station_pose_b =
    create_pose_facing_target({-0.2, 0.1, 2.0}, {0.0, 0.0, 0.0});

  // First run: one station.
  const auto [e0, a0] = make_measurement(station_pose_a);
  uut->addSample(e0, a0, /*station_id=*/ 0, /*deck_pose_id=*/ 0);
  const auto result_a = uut->solve();
  EXPECT_EQ(result_a.station_poses.size(), 1u);

  // Reset and run again with a different station.
  uut->reset();
  const auto [e1, a1] = make_measurement(station_pose_b);
  uut->addSample(e1, a1, /*station_id=*/ 7, /*deck_pose_id=*/ 3);
  NetworkNodePoses result_b;
  ASSERT_NO_THROW(result_b = uut->solve());
  EXPECT_EQ(result_b.station_poses.size(), 1u);
  EXPECT_TRUE(result_b.station_poses.count(7));
  EXPECT_TRUE(result_b.deck_poses.count(3));
  // IDs from the first run must not leak into the second.
  EXPECT_FALSE(result_b.station_poses.count(0));
  EXPECT_FALSE(result_b.deck_poses.count(0));
}

// ---------- Parameterized pose recovery tests ----------

struct InitializationScenario
{
  /// Station poses in world frame.
  std::vector<Sophus::SE3d> station_poses;
  /// Deck poses in world frame. The first deck is the root (identity in
  /// result).
  std::vector<Sophus::SE3d> deck_poses;
  /// Observations as (deck_id, station_id) pairs. The graph must be connected.
  std::vector<std::pair<std::size_t, std::size_t>> observations;
  std::string description;
};

class StationGeometryInitializationPoseTest
  : public ::testing::TestWithParam<InitializationScenario>
{
protected:
  static constexpr double kTranslationTolerance = 1e-3;  // meters
  static constexpr double kDirectionTolerance = 1e-2;  // radians
};

TEST_P(
  StationGeometryInitializationPoseTest,
  PosesMatchTruthRelativeToRootDeck) {
  const auto & scenario = GetParam();
  StationGeometryInitialization uut;

  // Feed observations as specified in the scenario.
  for (const auto &[deck_id, station_id] : scenario.observations) {
    // Express station pose in this deck's frame.
    const Sophus::SE3d station_in_deck =
      scenario.deck_poses[deck_id].inverse() *
      scenario.station_poses[station_id];
    const auto [elevations, azimuths] = compute_expected_measurements(
      station_in_deck);
    uut.addSample(elevations, azimuths, station_id, deck_id);
  }

  NetworkNodePoses result;
  ASSERT_NO_THROW(result = uut.solve()) << scenario.description;

  const Sophus::SE3d & root_deck = scenario.deck_poses.front();

  // Check deck poses
  for (DeckPoseId deck_id = 0;
    deck_id < scenario.deck_poses.size(); ++deck_id)
  {
    ASSERT_TRUE(result.deck_poses.count(deck_id))
      << "Missing deck " << deck_id << " in " << scenario.description;

    const Sophus::SE3d expected =
      root_deck.inverse() * scenario.deck_poses[deck_id];
    const Sophus::SE3d & recovered = result.deck_poses.at(deck_id);

    EXPECT_NEAR(
      compute_translation_error(recovered, expected), 0.0,
      kTranslationTolerance)
      << "Deck " << deck_id << " translation error in "
      << scenario.description;
    EXPECT_NEAR(
      compute_direction_error_radians(recovered, expected), 0.0,
      kDirectionTolerance)
      << "Deck " << deck_id << " direction error in " << scenario.description;
  }

  // Check station poses
  for (StationId station_id = 0;
    station_id < scenario.station_poses.size(); ++station_id)
  {
    ASSERT_TRUE(result.station_poses.count(station_id))
      << "Missing station " << station_id << " in " << scenario.description;

    const Sophus::SE3d expected =
      root_deck.inverse() * scenario.station_poses[station_id];
    const Sophus::SE3d & recovered = result.station_poses.at(station_id);

    EXPECT_NEAR(
      compute_translation_error(recovered, expected), 0.0,
      kTranslationTolerance)
      << "Station " << station_id << " translation error in "
      << scenario.description;
    EXPECT_NEAR(
      compute_direction_error_radians(recovered, expected), 0.0,
      kDirectionTolerance)
      << "Station " << station_id << " direction error in "
      << scenario.description;
  }
}

INSTANTIATE_TEST_SUITE_P(
  Scenarios, StationGeometryInitializationPoseTest,
  ::testing::Values(
    // Single deck, single station
    InitializationScenario{
    /* station_poses = */
    {create_pose_facing_target({0.3, 0.2, 1.5}, {0.0, 0.0, 0.0})},
    /* deck_poses = */ {Sophus::SE3d{}},
    /* observations = */ {{0, 0}},
    /* description = */ "OneDeckOneStation"},

    // Two decks (one closer, one farther) sharing a single station
    InitializationScenario{
    /* station_poses = */
    {create_pose_facing_target({0.0, 0.0, 2.0}, {0.0, 0.0, 0.0})},
    /* deck_poses = */
    {Sophus::SE3d{},
      Sophus::SE3d(Sophus::SO3d{}, Eigen::Vector3d(0.5, 0.0, 0.0))},
    /* observations = */ {{0, 0}, {1, 0}},
    /* description = */ "TwoDecksOneStation"},

    // One deck observing two stations at different positions
    InitializationScenario{
    /* station_poses = */
    {create_pose_facing_target({0.3, 0.2, 1.5}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({-0.3, -0.2, 2.0}, {0.0, 0.0, 0.0})},
    /* deck_poses = */ {Sophus::SE3d{}},
    /* observations = */ {{0, 0}, {0, 1}},
    /* description = */ "OneDeckTwoStations"},

    // Three stations A(0), B(1), C(2): two decks between A-B, three between
    // B-C
    InitializationScenario{
    /* station_poses = */
    {create_pose_facing_target({0.3, 0.2, 1.5}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({-0.3, 0.1, 1.8}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({0.0, -0.3, 2.0}, {0.0, 0.0, 0.0})},
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
    /* description = */
    "ThreeStationsABC_TwoBetweenAB_ThreeBetweenBC"},

    // Three stations A(0), B(1), C(2): one deck between A-B, two between
    // B-C, three between C-A
    InitializationScenario{
    /* station_poses = */
    {create_pose_facing_target({0.3, 0.2, 1.5}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({-0.3, 0.1, 1.8}, {0.0, 0.0, 0.0}),
      create_pose_facing_target({0.0, -0.3, 2.0}, {0.0, 0.0, 0.0})},
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
    "ThreeStationsABC_OneBetweenAB_TwoBetweenBC_ThreeBetweenCA"}));

}  // namespace lighthouse_geometry_utils
