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

// Test ceiling-mounted station pointing straight down
#include <gtest/gtest.h>
#include "lighthouse_geometry_utils/station_pose_pnp_solver.hpp"
#include "test_helpers.hpp"

namespace lighthouse_geometry_utils
{

TEST(CeilingStationTest, StationPointingStraightDown) {
  // Station at (0, 0, 2.0) looking at (0, 0, 0) - ceiling mounted pointing down
  const Sophus::SE3d station_in_deck_truth =
    test::create_pose_facing_target(
    Eigen::Vector3d(0.0, 0.0, 2.0),
    Eigen::Vector3d(0.0, 0.0, 0.0));

  std::cerr << "\n=== GROUND TRUTH (Ceiling Station) ===" << std::endl;
  std::cerr << "Position: " << station_in_deck_truth.translation().transpose() << std::endl;
  std::cerr << "X-axis: " << station_in_deck_truth.rotationMatrix().col(0).transpose() << std::endl;
  std::cerr << "Y-axis: " << station_in_deck_truth.rotationMatrix().col(1).transpose() << std::endl;
  std::cerr << "Z-axis: " << station_in_deck_truth.rotationMatrix().col(2).transpose() << std::endl;

  const auto [elevations, azimuths] =
    test::compute_expected_measurements(station_in_deck_truth);

  StationPosePnPSolver solver;
  const Sophus::SE3d station_in_deck_recovered = solver.solve(elevations, azimuths);

  std::cerr << "\n=== RECOVERED ===" << std::endl;
  std::cerr << "Position: " << station_in_deck_recovered.translation().transpose() << std::endl;
  std::cerr << "X-axis: " << station_in_deck_recovered.rotationMatrix().col(0).transpose() <<
    std::endl;

  const double trans_error = test::compute_translation_error(
    station_in_deck_recovered,
    station_in_deck_truth);
  const double dir_error = test::compute_direction_error_radians(
    station_in_deck_recovered,
    station_in_deck_truth);

  std::cerr << "Translation error: " << trans_error << " m" << std::endl;
  std::cerr << "Direction error: " << dir_error << " rad (" << (dir_error * 180.0 / M_PI) <<
    " deg)" << std::endl;

  EXPECT_LT(trans_error, 0.025);
  EXPECT_LT(dir_error, 2.0 * M_PI / 180.0);
}

}  // namespace lighthouse_geometry_utils
