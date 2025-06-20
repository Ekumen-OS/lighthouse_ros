// Copyright 2025 Ekumen, Inc.
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
#include <math.h>

#include <lighthouse_sensor_model/lighthouse_sensor_model.hpp>

namespace beluga {

namespace {

struct LightHouseSensorModel2DTests : public ::testing::Test {
  LighthouseStationMapType base_stations_map_;
  LighthouseSensorModelParam params_;

  void SetUp() override {
    // setup the map with two workstations
    base_stations_map_.station_poses_in_world[0] = Sophus::SE3d(
        Sophus::SO3d::rotZ(M_PI / 4.0), Eigen::Vector3d(0.0, 0.0, 1.0));

    base_stations_map_.station_poses_in_world[1] = Sophus::SE3d(
        Sophus::SO3d::rotZ(-M_PI / 2.0), Eigen::Vector3d(5.0, 10.0, 1.0));

    // setup the sensor model parameters
    params_.sigma_angular_error = 0.02; // assume very narrow error margins
    params_.sensor_height = 0.5;
  }

  LighthouseDirectionalDetection calculate_detection(const Sophus::SE2d &state,
                                                     const size_t station_id) {
    // convert 2D pose to 3D pose, assuming the robot is on the ground
    const Sophus::SE3d deck_pose_in_world{
        Sophus::SO3d::rotZ(state.so2().log()),
        Eigen::Vector3d{state.translation().x(), state.translation().y(),
                        params_.sensor_height}};

    double sensor_x_distance_{0.0296};
    double sensor_y_distance_{0.0150};

    // calculate deck sensor poses in the world frame
    const auto sensor_offsets = std::vector<Eigen::Vector3d>{
        Eigen::Vector3d{-sensor_x_distance_ / 2.0, sensor_y_distance_ / 2.0,
                        0.0},
        Eigen::Vector3d{-sensor_x_distance_ / 2.0, -sensor_y_distance_ / 2.0,
                        0.0},
        Eigen::Vector3d{sensor_x_distance_ / 2.0, sensor_y_distance_ / 2.0,
                        0.0},
        Eigen::Vector3d{sensor_x_distance_ / 2.0, -sensor_y_distance_ / 2.0,
                        0.0},
    };

    LighthouseDirectionalDetection detection;
    detection.base_station_id = station_id;

    for (size_t sensor_id = 0; sensor_id < sensor_offsets.size(); ++sensor_id) {
      const auto sensor_pose_in_world =
          deck_pose_in_world *
          Sophus::SE3d{Sophus::SO3d{}, sensor_offsets[sensor_id]};

      const auto sensor_pose_in_station =
          base_stations_map_.station_poses_in_world.at(station_id).inverse() *
          sensor_pose_in_world;

      // TODO we have lots of doubts regarding how the angles are measured,
      // we need to revisit this with live measurements
      detection.deck_sensor_angles[sensor_id].azimuth =
          std::atan2(sensor_pose_in_station.translation().y(),
                     sensor_pose_in_station.translation().x());

      detection.deck_sensor_angles[sensor_id].elevation =
          std::atan2(sensor_pose_in_station.translation().z(),
                     sensor_pose_in_station.translation().norm());
    }

    return detection;
  }

  auto build_test_pose(const double x, const double y, const double theta) {
    return Sophus::SE2d{Sophus::SO2d::exp(theta), Eigen::Vector2d{x, y}};
  }
};

TEST_F(LightHouseSensorModel2DTests, ConstructionDoesThrow) {
  ASSERT_NO_THROW(
      { LighthouseSensorModel2D model(params_, base_stations_map_); });
}

TEST_F(LightHouseSensorModel2DTests, ChangingMapDoesNotThrow) {
  ASSERT_NO_THROW({
    LighthouseSensorModel2D model(params_, base_stations_map_);
    model.update_map(std::move(base_stations_map_));
  });
}

TEST_F(LightHouseSensorModel2DTests, CalculateWeight) {
  constexpr double kTolerance = 1e-2;

  // create a list of test poses
  const std::vector<Sophus::SE2d> test_poses{
      build_test_pose(5.0, 0.0, 0.75 * M_PI), // upper-right, facing center
      build_test_pose(2.5, 2.5, 0.0),         // center, facing forward
      build_test_pose(2.5, 4.0, -M_PI / 2.0), // center-left, facing right
      build_test_pose(2.5, 2.5, M_PI),        // center, facing backward
  };

  LighthouseSensorModel2D model(params_, base_stations_map_);

  //
  // the exact pose and orientation should have weights close to 1.0

  for (const std::size_t station_id : {0, 1}) {
    for (const auto &pose : test_poses) {
      const auto weight_function = model(calculate_detection(pose, station_id));
      const auto weight = weight_function(pose);
      ASSERT_NEAR(weight, 1.0, kTolerance);
    }
  }

  //
  // offset poses around the true pose should have weights close to 0.0

  // create a list of offsets
  const std::vector<Sophus::SE2d> offsets{
      build_test_pose(+1.0, 0.0, 0.0), //
      build_test_pose(-1.0, 0.0, 0.0), //
      build_test_pose(0.0, +1.0, 0.0), //
      build_test_pose(0.0, -1.0, 0.0), //
  };

  // each test pose and each offset, calculate the
  // weight. for the first offset ("no error"), the weight should be almost 1.0,
  // and for the rest of the offsets, the weight should be almost 0.0
  for (const std::size_t station_id : {0, 1}) {
    for (const auto &true_pose : test_poses) {
      for (const auto &offset : offsets) {
        const auto offset_hyphothesis = true_pose * offset;
        // calculate the measurements for the real pose, and
        // calculate the weight for the offset pose
        const auto weight_function =
            model(calculate_detection(true_pose, station_id));
        const auto weight = weight_function(offset_hyphothesis);
        ASSERT_NEAR(weight, 0.0, kTolerance);
      }
    }
  }
}

TEST_F(LightHouseSensorModel2DTests, UnmappedStationsReturnOneEverywhere) {
  LighthouseSensorModel2D model(params_, base_stations_map_);

  auto detection = LighthouseDirectionalDetection{};
  detection.base_station_id = 2;

  const auto weight_function = model(std::move(detection));

  const std::vector<Sophus::SE2d> test_poses{
      build_test_pose(5.0, 0.0, 0.75 * M_PI), // upper-right, facing center
      build_test_pose(2.5, 2.5, 0.0),         // center, facing forward
      build_test_pose(2.5, 4.0, -M_PI / 2.0), // center-left, facing right
      build_test_pose(2.5, 2.5, M_PI),        // center, facing backward
  };

  for (const auto &pose : test_poses) {
    const auto weight = weight_function(pose);
    EXPECT_NEAR(weight, 1.0, 1e-3);
  }
}

} // namespace

} // namespace beluga
