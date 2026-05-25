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
#include "lighthouse_geometry_utils/station_geometry_optimization.hpp"
#include "lighthouse_geometry_utils/deck_pose_optimization.hpp"
#include "lighthouse_geometry_utils/station_pose_pnp_solver.hpp"
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <iostream>
#include <iomanip>

using namespace lighthouse_geometry_utils;

TEST(RealDataCollinearity, VerifyAxesAlignment)
{
  // Data from screenshot - convert degrees to radians
  const double deg_to_rad = M_PI / 180.0;

  // Sample 0 (Station ID 1, Deck Pose 0)
  std::array<double, 4> sample0_az = {
    2.5012 * deg_to_rad, 2.6780 * deg_to_rad,
    2.4870 * deg_to_rad, 2.6593 * deg_to_rad
  };
  std::array<double, 4> sample0_el = {
    -19.3877 * deg_to_rad, -19.3939 * deg_to_rad,
    -19.4964 * deg_to_rad, -19.5037 * deg_to_rad
  };

  // Sample 1 (Station ID 1, Deck Pose 1)
  std::array<double, 4> sample1_az = {
    6.4060 * deg_to_rad, 6.5889 * deg_to_rad,
    6.3944 * deg_to_rad, 6.5720 * deg_to_rad
  };
  std::array<double, 4> sample1_el = {
    -19.9170 * deg_to_rad, -19.9209 * deg_to_rad,
    -20.0298 * deg_to_rad, -20.0360 * deg_to_rad
  };

  // Sample 2 (Station ID 1, Deck Pose 2)
  std::array<double, 4> sample2_az = {
    2.2294 * deg_to_rad, 2.4175 * deg_to_rad,
    2.2103 * deg_to_rad, 2.3924 * deg_to_rad
  };
  std::array<double, 4> sample2_el = {
    -20.3036 * deg_to_rad, -20.3131 * deg_to_rad,
    -20.4225 * deg_to_rad, -20.4320 * deg_to_rad
  };

  std::cout << "\n=== Real Data Collinearity Test ===" << std::endl;
  std::cout << "Testing station and deck axes alignment from screenshot data" << std::endl;

  // Step 0: Check PnP initialization for sample 0
  std::cout << "\n--- PnP Initialization (Sample 0) ---" << std::endl;
  StationPosePnPSolver pnp_solver;
  Sophus::SE3d station_in_deck_pnp = pnp_solver.solve(sample0_el, sample0_az);

  std::cout << "Station-in-deck from PnP:" << std::endl;
  std::cout << "Translation: ["
            << station_in_deck_pnp.translation().x() << ", "
            << station_in_deck_pnp.translation().y() << ", "
            << station_in_deck_pnp.translation().z() << "]" << std::endl;

  const auto pnp_q = station_in_deck_pnp.unit_quaternion();
  std::cout << "Quaternion: ["
            << pnp_q.x() << ", " << pnp_q.y() << ", "
            << pnp_q.z() << ", " << pnp_q.w() << "]" << std::endl;

  // Since deck 0 is at origin/identity, station pose in world = station in deck
  const Eigen::Matrix3d pnp_R = station_in_deck_pnp.rotationMatrix();
  Eigen::Vector3d pnp_x = pnp_R.col(0);
  Eigen::Vector3d pnp_y = pnp_R.col(1);
  Eigen::Vector3d pnp_z = pnp_R.col(2);

  std::cout << "X-axis: [" << pnp_x.transpose() << "]" << std::endl;
  std::cout << "Y-axis: [" << pnp_y.transpose() << "]" << std::endl;
  std::cout << "Z-axis: [" << pnp_z.transpose() << "]" << std::endl;

  // Check if PnP already has the bug (axes should be collinear with identity)
  double pnp_x_align = std::abs(pnp_x.dot(Eigen::Vector3d::UnitX()));
  double pnp_y_align = std::abs(pnp_y.dot(Eigen::Vector3d::UnitY()));
  double pnp_z_align = std::abs(pnp_z.dot(Eigen::Vector3d::UnitZ()));

  std::cout << "PnP alignment with identity frame:" << std::endl;
  std::cout << "  |X · [1,0,0]| = " << pnp_x_align << std::endl;
  std::cout << "  |Y · [0,1,0]| = " << pnp_y_align << std::endl;
  std::cout << "  |Z · [0,0,1]| = " << pnp_z_align << std::endl;

  // Step 1: Solve for station geometry
  StationGeometryOptimization station_optimizer;
  station_optimizer.addSample(sample0_el, sample0_az, 1, 0);
  station_optimizer.addSample(sample1_el, sample1_az, 1, 1);
  station_optimizer.addSample(sample2_el, sample2_az, 1, 2);

  auto station_result = station_optimizer.solve();

  ASSERT_EQ(station_result.station_ids.size(), 1);
  ASSERT_EQ(station_result.station_ids[0], 1);

  const auto & station_pose = station_result.station_poses[0];

  std::cout << "\n--- Station Pose (ID 1) AFTER OPTIMIZATION ---" << std::endl;
  std::cout << "Translation: ["
            << station_pose.translation().x() << ", "
            << station_pose.translation().y() << ", "
            << station_pose.translation().z() << "]" << std::endl;

  const auto station_q = station_pose.unit_quaternion();
  std::cout << "Quaternion: ["
            << station_q.x() << ", " << station_q.y() << ", "
            << station_q.z() << ", " << station_q.w() << "]" << std::endl;

  const Eigen::Matrix3d station_R = station_pose.rotationMatrix();
  Eigen::Vector3d station_x = station_R.col(0);
  Eigen::Vector3d station_y = station_R.col(1);
  Eigen::Vector3d station_z = station_R.col(2);

  std::cout << "X-axis: [" << station_x.transpose() << "]" << std::endl;
  std::cout << "Y-axis: [" << station_y.transpose() << "]" << std::endl;
  std::cout << "Z-axis: [" << station_z.transpose() << "]" << std::endl;

  // Step 2: Solve for deck poses using DeckPoseOptimization (same as mapper)
  DeckPoseOptimization deck_optimizer(station_result.station_poses, station_result.station_ids);

  std::vector<Sophus::SE3d> deck_poses;
  std::vector<std::array<double, 4>> all_samples_el = {sample0_el, sample1_el, sample2_el};
  std::vector<std::array<double, 4>> all_samples_az = {sample0_az, sample1_az, sample2_az};

  for (size_t i = 0; i < 3; ++i) {
    std::vector<DeckPoseOptimization::Sample> deck_samples;
    deck_samples.push_back({all_samples_el[i], all_samples_az[i], 1});

    auto [deck_pose, deck_cov] = deck_optimizer.solve(deck_samples);
    deck_poses.push_back(deck_pose);

    std::cout << "\n--- Deck Pose " << i << " ---" << std::endl;
    std::cout << "Translation: ["
              << deck_pose.translation().x() << ", "
              << deck_pose.translation().y() << ", "
              << deck_pose.translation().z() << "]" << std::endl;

    const auto deck_q = deck_pose.unit_quaternion();
    std::cout << "Quaternion: ["
              << deck_q.x() << ", " << deck_q.y() << ", "
              << deck_q.z() << ", " << deck_q.w() << "]" << std::endl;

    const Eigen::Matrix3d deck_R = deck_pose.rotationMatrix();
    Eigen::Vector3d deck_x = deck_R.col(0);
    Eigen::Vector3d deck_y = deck_R.col(1);
    Eigen::Vector3d deck_z = deck_R.col(2);

    std::cout << "X-axis: [" << deck_x.transpose() << "]" << std::endl;
    std::cout << "Y-axis: [" << deck_y.transpose() << "]" << std::endl;
    std::cout << "Z-axis: [" << deck_z.transpose() << "]" << std::endl;

    // Check collinearity with station axes
    double x_alignment = std::abs(deck_x.dot(station_x));
    double y_alignment = std::abs(deck_y.dot(station_y));
    double z_alignment = std::abs(deck_z.dot(station_z));

    std::cout << "\n  Axis alignment with station:" << std::endl;
    std::cout << "    |deck_X · station_X| = " << x_alignment
              << (x_alignment > 0.95 ? " ✓" : " ✗ NOT COLLINEAR") << std::endl;
    std::cout << "    |deck_Y · station_Y| = " << y_alignment
              << (y_alignment > 0.95 ? " ✓" : " ✗ NOT COLLINEAR") << std::endl;
    std::cout << "    |deck_Z · station_Z| = " << z_alignment
              << (z_alignment > 0.95 ? " ✓" : " ✗ NOT COLLINEAR") << std::endl;

    // Also check cross-alignments to detect rotation errors
    double x_y_cross = std::abs(deck_x.dot(station_y));
    double x_z_cross = std::abs(deck_x.dot(station_z));
    double y_x_cross = std::abs(deck_y.dot(station_x));
    double y_z_cross = std::abs(deck_y.dot(station_z));
    double z_x_cross = std::abs(deck_z.dot(station_x));
    double z_y_cross = std::abs(deck_z.dot(station_y));

    std::cout << "  Cross-alignments (should be ~0):" << std::endl;
    std::cout << "    |deck_X · station_Y| = " << x_y_cross << std::endl;
    std::cout << "    |deck_X · station_Z| = " << x_z_cross << std::endl;
    std::cout << "    |deck_Y · station_X| = " << y_x_cross << std::endl;
    std::cout << "    |deck_Y · station_Z| = " << y_z_cross << std::endl;
    std::cout << "    |deck_Z · station_X| = " << z_x_cross << std::endl;
    std::cout << "    |deck_Z · station_Y| = " << z_y_cross << std::endl;

    // Detect if there's a 180° rotation around X (Y and Z would swap)
    if (x_alignment > 0.95 && y_z_cross > 0.9 && z_y_cross > 0.9) {
      std::cout << "\n  ⚠️  DETECTED: 180° rotation around X-axis!" << std::endl;
      std::cout << "      Deck Y-axis aligns with station Z-axis" << std::endl;
      std::cout << "      Deck Z-axis aligns with station Y-axis" << std::endl;
    }

    // Expect collinearity
    EXPECT_GT(x_alignment, 0.95) << "Deck " << i << " X-axis not collinear with station X";
    EXPECT_GT(y_alignment, 0.95) << "Deck " << i << " Y-axis not collinear with station Y";
    EXPECT_GT(z_alignment, 0.95) << "Deck " << i << " Z-axis not collinear with station Z";
  }

  std::cout << "\n=== Summary ===" << std::endl;
  std::cout << "If axes are NOT collinear, this indicates the orientation bug!" << std::endl;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
