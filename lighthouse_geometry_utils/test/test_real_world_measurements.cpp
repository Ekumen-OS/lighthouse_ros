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

#include <array>
#include <vector>

#include "lighthouse_geometry_utils/station_geometry_initialization.hpp"
#include "lighthouse_geometry_utils/station_geometry_optimization.hpp"
#include "lighthouse_geometry_utils/station_pose_pnp_solver.hpp"
#include "lighthouse_geometry_utils/utils.hpp"
#include "test_helpers.hpp"

#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils
{

// Real-world measurements captured from /lighthouse topic
// Source: sample_lighthouse_messages.yaml and sample_lighthouse_messages_new.yaml
// Station ID: 2

namespace
{
constexpr int kStationId = 2;
constexpr double kDegToRad = M_PI / 180.0;

// Dataset 1: Original measurements (timestamp: sec=1776575734)
// Ground truth from PnP solver: position (0.461, 0.003, 0.207) meters, yaw -176.096°
// Expected yaw orientation: ~180° (station faces deck, deck front points away)
constexpr double kDataset1_Azimuth0_deg_1 = -3.792601880877746;
constexpr double kDataset1_Azimuth1_deg_1 = -1.9967398119122124;
constexpr double kDataset1_Azimuth2_deg_1 = -3.747084639498448;
constexpr double kDataset1_Azimuth3_deg_1 = -1.875235109717863;

constexpr double kDataset1_Elevation0_deg_1 = -20.95986173994102;
constexpr double kDataset1_Elevation1_deg_1 = -20.89666082616373;
constexpr double kDataset1_Elevation2_deg_1 = -22.57548182239643;
constexpr double kDataset1_Elevation3_deg_1 = -22.5139699281217;

constexpr double kDataset1_Azimuth0_deg_2 = -3.7858307210031144;
constexpr double kDataset1_Azimuth1_deg_2 = -1.9982445141065948;
constexpr double kDataset1_Azimuth2_deg_2 = -3.749341692789984;
constexpr double kDataset1_Azimuth3_deg_2 = -1.8737304075235315;

constexpr double kDataset1_Elevation0_deg_2 = -20.97426701320246;
constexpr double kDataset1_Elevation1_deg_2 = -20.90553499188374;
constexpr double kDataset1_Elevation2_deg_2 = -22.583031349904456;
constexpr double kDataset1_Elevation3_deg_2 = -22.51504965452773;

// Dataset 2: New measurements (timestamp: sec=1776576997)
// Station position: ~50cm away at 45° to the right of X axis, similar Z height as dataset 1
// Ground truth from PnP solver: position (0.359, -0.352, 0.236) meters, yaw 135.042°
// Expected yaw orientation: ~135° (station faces deck from different angle than dataset 1)
constexpr double kDataset2_Azimuth0_deg_1 = 0.6752351097178613;
constexpr double kDataset2_Azimuth1_deg_1 = 1.8714733542319706;
constexpr double kDataset2_Azimuth2_deg_1 = -2.044514106583071;
constexpr double kDataset2_Azimuth3_deg_1 = -0.8937931034482852;

constexpr double kDataset2_Elevation0_deg_1 = -20.88501157100074;
constexpr double kDataset2_Elevation1_deg_1 = -21.344877359890656;
constexpr double kDataset2_Elevation2_deg_1 = -21.869028233989916;
constexpr double kDataset2_Elevation3_deg_1 = -22.393452813546496;

constexpr double kDataset2_Azimuth0_deg_2 = 0.6744827586206829;
constexpr double kDataset2_Azimuth1_deg_2 = 1.870344827586203;
constexpr double kDataset2_Azimuth2_deg_2 = -2.0467711598746066;
constexpr double kDataset2_Azimuth3_deg_2 = -0.8952978056426422;

constexpr double kDataset2_Elevation0_deg_2 = -20.88279241914026;
constexpr double kDataset2_Elevation1_deg_2 = -21.343225353552075;
constexpr double kDataset2_Elevation2_deg_2 = -21.870119983650145;
constexpr double kDataset2_Elevation3_deg_2 = -22.39561678201468;

struct PnPTestData
{
  const char * name;
  std::array<double, 4> elevations;
  std::array<double, 4> azimuths;
  Sophus::SE3d ground_truth_pose;
};

// Hardcoded ground truth poses captured from real measurements
// Dataset 1: position (0.461, 0.003, 0.207) meters, yaw -176.096°
const Sophus::SE3d kDataset1GroundTruthPose(
  Sophus::SO3d(
    (Eigen::Matrix3d() <<
      -0.99672451220208091, 0.06933418012488736, -0.041629535692023413,
      -0.068028432314418644, -0.99716990273161499, -0.03200496028843007,
      -0.043730757738305155, -0.029068136380426941, 0.99862038046246715
    ).finished()
  ),
  Eigen::Vector3d(
    0.461232836210759, 0.0026035798453285242, 0.20654005374398468)
);

// Dataset 2: position (0.359, -0.352, 0.236) meters, yaw 135.042°
const Sophus::SE3d kDataset2GroundTruthPose(
  Sophus::SO3d(
    (Eigen::Matrix3d() <<
      -0.70631376928093781, -0.70748654587174487, -0.024158781728849093,
      0.70527240135800173, -0.70622033961170327, 0.061997353180994086,
      -0.060923716292364941, 0.026751062206920237, 0.99778388514944905
    ).finished()
  ),
  Eigen::Vector3d(
    0.35939651251571414, -0.35202193293210149, 0.23569802160408876)
);

// Create test data instances
PnPTestData create_dataset1_pnp_test()
{
  std::array<double, 4> elevations = {
    kDataset1_Elevation0_deg_1 * kDegToRad,
    kDataset1_Elevation1_deg_1 * kDegToRad,
    kDataset1_Elevation2_deg_1 * kDegToRad,
    kDataset1_Elevation3_deg_1 * kDegToRad
  };

  std::array<double, 4> azimuths = {
    kDataset1_Azimuth0_deg_1 * kDegToRad,
    kDataset1_Azimuth1_deg_1 * kDegToRad,
    kDataset1_Azimuth2_deg_1 * kDegToRad,
    kDataset1_Azimuth3_deg_1 * kDegToRad
  };

  return PnPTestData{
    "Dataset1_Original",
    elevations,
    azimuths,
    kDataset1GroundTruthPose
  };
}

PnPTestData create_dataset2_pnp_test()
{
  std::array<double, 4> elevations = {
    kDataset2_Elevation0_deg_1 * kDegToRad,
    kDataset2_Elevation1_deg_1 * kDegToRad,
    kDataset2_Elevation2_deg_1 * kDegToRad,
    kDataset2_Elevation3_deg_1 * kDegToRad
  };

  std::array<double, 4> azimuths = {
    kDataset2_Azimuth0_deg_1 * kDegToRad,
    kDataset2_Azimuth1_deg_1 * kDegToRad,
    kDataset2_Azimuth2_deg_1 * kDegToRad,
    kDataset2_Azimuth3_deg_1 * kDegToRad
  };

  return PnPTestData{
    "Dataset2_NewCapture",
    elevations,
    azimuths,
    kDataset2GroundTruthPose
  };
}

const std::vector<PnPTestData> kPnPTestData = {
  create_dataset1_pnp_test(),
  create_dataset2_pnp_test()
};

}  // namespace

class PnPSolverTest : public ::testing::TestWithParam<PnPTestData>
{
};

TEST_P(PnPSolverTest, RecoversPoseFromSingleMeasurement)
{
  const auto & test_data = GetParam();

  StationPosePnPSolver solver;
  const Sophus::SE3d recovered_pose = solver.solve(
    test_data.elevations,
    test_data.azimuths);

  // Check translation error
  const double translation_error =
    test::compute_translation_error(
    recovered_pose, test_data.ground_truth_pose);

  // Check orientation error
  const double direction_error =
    test::compute_direction_error_radians(
    recovered_pose, test_data.ground_truth_pose);

  // Compute component-wise errors
  const Eigen::Vector3d pos_error =
    recovered_pose.translation() -
    test_data.ground_truth_pose.translation();
  const Eigen::Vector3d expected_pos =
    test_data.ground_truth_pose.translation();
  const Eigen::Vector3d calculated_pos =
    recovered_pose.translation();

  // Print detailed errors
  std::cout << "\n=== " << test_data.name << " ===" << std::endl;
  std::cout << "Position:" << std::endl;
  std::cout << "  X: expected=" << expected_pos.x() * 1000.0
            << " mm, calculated="
            << calculated_pos.x() * 1000.0
            << " mm, error=" << pos_error.x() * 1000.0
            << " mm" << std::endl;
  std::cout << "  Y: expected=" << expected_pos.y() * 1000.0
            << " mm, calculated="
            << calculated_pos.y() * 1000.0
            << " mm, error=" << pos_error.y() * 1000.0
            << " mm" << std::endl;
  std::cout << "  Z: expected=" << expected_pos.z() * 1000.0
            << " mm, calculated="
            << calculated_pos.z() * 1000.0
            << " mm, error=" << pos_error.z() * 1000.0
            << " mm" << std::endl;
  std::cout << "  Total translation error: "
            << translation_error * 1000.0
            << " mm" << std::endl;
  std::cout << "Orientation:" << std::endl;
  std::cout << "  Direction error: "
            << direction_error * 180.0 / M_PI
            << " degrees" << std::endl;

  // Validate against ground truth tolerances
  EXPECT_LT(translation_error, 0.025)
    << "Translation error: " << translation_error << " m";

  EXPECT_LT(direction_error, 0.052)  // ~3 degrees in radians
    << "Direction error: " << direction_error << " rad ("
    << direction_error * 180.0 / M_PI << " deg)";
}

INSTANTIATE_TEST_SUITE_P(
  RealWorldMeasurements,
  PnPSolverTest,
  ::testing::ValuesIn(kPnPTestData),
  [](const ::testing::TestParamInfo<PnPTestData> & info) {
    return info.param.name;
  });

// Test using optimization solver
class OptimizationSolverTest : public ::testing::TestWithParam<PnPTestData>
{
};

TEST_P(OptimizationSolverTest, RecoversPoseFromSingleMeasurement)
{
  const auto & test_data = GetParam();

  StationGeometryOptimization optimizer;
  optimizer.addSample(
    test_data.elevations,
    test_data.azimuths,
    kStationId,
    0  // deck_pose_id
  );

  const auto result = optimizer.solve();

  // Should have recovered one station
  ASSERT_EQ(result.station_poses.size(), 1);
  ASSERT_EQ(result.station_ids.size(), 1);
  EXPECT_EQ(result.station_ids[0], kStationId);

  const Sophus::SE3d & recovered_pose = result.station_poses[0];

  // Check translation error
  const double translation_error =
    test::compute_translation_error(
    recovered_pose, test_data.ground_truth_pose);

  // Check orientation error
  const double direction_error =
    test::compute_direction_error_radians(
    recovered_pose, test_data.ground_truth_pose);

  // Compute component-wise errors
  const Eigen::Vector3d pos_error =
    recovered_pose.translation() -
    test_data.ground_truth_pose.translation();
  const Eigen::Vector3d expected_pos =
    test_data.ground_truth_pose.translation();
  const Eigen::Vector3d calculated_pos =
    recovered_pose.translation();

  // Print detailed errors
  std::cout << "\n=== " << test_data.name
            << " (Optimization) ===" << std::endl;
  std::cout << "Position:" << std::endl;
  std::cout << "  X: expected=" << expected_pos.x() * 1000.0
            << " mm, calculated="
            << calculated_pos.x() * 1000.0
            << " mm, error=" << pos_error.x() * 1000.0
            << " mm" << std::endl;
  std::cout << "  Y: expected=" << expected_pos.y() * 1000.0
            << " mm, calculated="
            << calculated_pos.y() * 1000.0
            << " mm, error=" << pos_error.y() * 1000.0
            << " mm" << std::endl;
  std::cout << "  Z: expected=" << expected_pos.z() * 1000.0
            << " mm, calculated="
            << calculated_pos.z() * 1000.0
            << " mm, error=" << pos_error.z() * 1000.0
            << " mm" << std::endl;
  std::cout << "  Total translation error: "
            << translation_error * 1000.0
            << " mm" << std::endl;
  std::cout << "Orientation:" << std::endl;
  std::cout << "  Direction error: "
            << direction_error * 180.0 / M_PI
            << " degrees" << std::endl;

  // Validate against ground truth tolerances
  EXPECT_LT(translation_error, 0.025)
    << "Translation error: " << translation_error << " m";

  EXPECT_LT(direction_error, 0.052)  // ~3 degrees in radians
    << "Direction error: " << direction_error << " rad ("
    << direction_error * 180.0 / M_PI << " deg)";
}

INSTANTIATE_TEST_SUITE_P(
  RealWorldMeasurements,
  OptimizationSolverTest,
  ::testing::ValuesIn(kPnPTestData),
  [](const ::testing::TestParamInfo<PnPTestData> & info) {
    return info.param.name;
  });

}  // namespace lighthouse_geometry_utils
