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

#include "lighthouse_geometry_utils/station_pose_pnp_solver.hpp"

#include <cmath>
#include <random>
#include <utility>

namespace lighthouse_geometry_utils {

// Inverse projection: compute azimuth/elevation angles from pose and sensor
// positions
std::pair<std::array<double, 4>, std::array<double, 4>>
compute_expected_measurements(const Sophus::SE3d &station_pose_in_deck,
                              const std::vector<cv::Point3d> &sensor_poses) {
  std::array<double, 4> elevations;
  std::array<double, 4> azimuths;

  const Sophus::SE3d deck_pose_in_station = station_pose_in_deck.inverse();

  for (std::size_t i = 0; i < sensor_poses.size(); ++i) {
    const Eigen::Vector3d sensor_translation_in_deck(
        sensor_poses[i].x, sensor_poses[i].y, sensor_poses[i].z);

    const Eigen::Vector3d sensor_translation_in_station =
        deck_pose_in_station * sensor_translation_in_deck;

    const double x = sensor_translation_in_station.x();
    const double y = sensor_translation_in_station.y();
    const double z = sensor_translation_in_station.z();

    azimuths[i] = std::atan2(y, x);
    elevations[i] = std::atan2(z, std::sqrt(x * x + y * y));
  }

  return {elevations, azimuths};
}

// Adds independent Gaussian noise to each angle measurement
std::pair<std::array<double, 4>, std::array<double, 4>>
compute_expected_measurements_with_noise(
    const Sophus::SE3d &station_pose_in_deck_frame,
    const std::vector<cv::Point3d> &sensor_poses, double elevation_noise_mean,
    double elevation_noise_stddev, double azimuth_noise_mean,
    double azimuth_noise_stddev, std::mt19937 &rng) {
  auto [elevations, azimuths] =
      compute_expected_measurements(station_pose_in_deck_frame, sensor_poses);

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

double compute_translation_error(const Sophus::SE3d &pose1,
                                 const Sophus::SE3d &pose2) {
  const Eigen::Vector3d translation_error =
      pose1.translation() - pose2.translation();
  return translation_error.norm();
}

double compute_direction_error_radians(const Sophus::SE3d &pose1,
                                       const Sophus::SE3d &pose2) {
  const Eigen::Vector3d optical_axis1 = pose1.rotationMatrix().col(0);
  const Eigen::Vector3d optical_axis2 = pose2.rotationMatrix().col(0);
  const double cos_angle = optical_axis1.dot(optical_axis2);
  return std::acos(std::clamp(cos_angle, -1.0, 1.0));
}

class StationPosePnPSolverPoseTest
    : public ::testing::TestWithParam<Sophus::SE3d> {
protected:
  void SetUp() override { uut = std::make_unique<StationPosePnPSolver>(); }

  void TearDown() override { uut.reset(); }

  std::unique_ptr<StationPosePnPSolver> uut;

  static constexpr double kMaxTranslationErrorMeters = 0.025;
  static constexpr double kMaxDirectionErrorRadians = 2.0 * M_PI / 180.0;

  // Noise needs to be very small, because at 3m distance the angular error
  // translates to errors larger than the distance between the sensors.
  static constexpr double kNoiseMeanRadians = 0.0;
  static constexpr double kNoiseStddevRadians = 0.001 * M_PI / 180.0;
};

TEST_P(StationPosePnPSolverPoseTest, recovers_pose_from_measurements) {
  const Sophus::SE3d expected_pose = GetParam();

  const double distance = expected_pose.translation().norm();
  ASSERT_GE(distance, 1.0) << "Station too close: " << distance << "m";
  ASSERT_LE(distance, 3.0) << "Station too far: " << distance << "m";

  const Sophus::SE3d deck_pose_in_station = expected_pose.inverse();
  const Eigen::Vector3d deck_origin_in_station =
      deck_pose_in_station * Eigen::Vector3d::Zero();
  ASSERT_GT(deck_origin_in_station.x(), 0.0)
      << "Deck not in front of station (X=" << deck_origin_in_station.x()
      << ")";

  std::mt19937 rng(42);

  const std::vector<cv::Point3d> sensor_poses{
      cv::Point3d(-0.01745, 0.0075, 0.0),  // back-left
      cv::Point3d(-0.01745, -0.0075, 0.0), // back-right
      cv::Point3d(0.01745, 0.0075, 0.0),   // front-left
      cv::Point3d(0.01745, -0.0075, 0.0)   // front-right
  };

  const auto [elevations, azimuths] = compute_expected_measurements_with_noise(
      expected_pose, sensor_poses, kNoiseMeanRadians, kNoiseStddevRadians,
      kNoiseMeanRadians, kNoiseStddevRadians, rng);

  const Sophus::SE3d recovered_pose = uut->calculate(elevations, azimuths);

  const double translation_error_norm =
      compute_translation_error(recovered_pose, expected_pose);
  EXPECT_LT(translation_error_norm, kMaxTranslationErrorMeters)
      << "Translation error: " << translation_error_norm << " m";

  const double direction_error_rad =
      compute_direction_error_radians(recovered_pose, expected_pose);
  EXPECT_LT(direction_error_rad, kMaxDirectionErrorRadians)
      << "Direction error: " << (direction_error_rad * 180.0 / M_PI)
      << " degrees";
}

INSTANTIATE_TEST_SUITE_P(
    VariousStationPoses, StationPosePnPSolverPoseTest,
    ::testing::Values(
        Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(0.679965, 0.000000,
                                                       3.067124)),
                     Eigen::Vector3d(0.906308, 0.000000, -0.422618)),
        Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(-0.425356, -0.138206,
                                                       -2.482636)),
                     Eigen::Vector3d(0.931555, 0.676814, -0.406499)),
        Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(-0.228311, -0.165878,
                                                       -1.874470)),
                     Eigen::Vector3d(0.432641, 1.331532, -0.346191)),
        Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(-0.091376, -0.125769,
                                                       -1.254298)),
                     Eigen::Vector3d(-0.508571, 1.565221, -0.241070)),
        Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(-0.015231, -0.046876,
                                                       -0.628194)),
                     Eigen::Vector3d(-1.522756, 1.106347, -0.091325)),
        Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(0.000000, 0.048481,
                                                       -0.000000)),
                     Eigen::Vector3d(-2.103082, 0.000000, 0.102040)),
        Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(-0.045692, 0.140624,
                                                       0.627196)),
                     Eigen::Vector3d(-1.862439, -1.353141, 0.337208)),
        Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(-0.152267, 0.209578,
                                                       1.250135)),
                     Eigen::Vector3d(-0.764288, -2.352237, 0.611568)),
        Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(-0.319431, 0.232080,
                                                       1.864396)),
                     Eigen::Vector3d(0.806832, -2.483175, 0.921743)),
        Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(-0.545954, 0.177391,
                                                       2.462637)),
                     Eigen::Vector3d(2.192323, -1.592816, 1.263629))));

} // namespace lighthouse_geometry_utils
