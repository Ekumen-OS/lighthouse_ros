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

#ifndef BELUGA_LIGHTHOUSE_SENSOR_MODEL_HPP
#define BELUGA_LIGHTHOUSE_SENSOR_MODEL_HPP

// external
#include <range/v3/view/all.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/so2.hpp>

// standard library
#include <array>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <utility>

/**
 * \file
 * \brief Implementation of a simple sensor model that works with Lighthouse
 * deck data.
 */

namespace beluga {

/// Parameters used to construct a LandmarkSensorModel instance (both 2D and
/// 3D).
struct LighthouseSensorModelParam {
  double sigma_angular_error{1.0}; ///< Standard deviation of the angular error.
  double sensor_height{0.0}; ///< Height of the deck above the robot footprint.
};

struct LighthouseStationMapType {
  /// @brief Map of station poses in the world frame.
  std::unordered_map<std::size_t, Sophus::SE3d> station_poses_in_world;
};

struct LighthouseSensorDirectionAngles {
  double azimuth;   /// angle in radians relative to the base station
  double elevation; /// angle in radians relative to the base station
};

struct LighthouseDirectionalDetection {
  std::size_t base_station_id; // id of the base station
  std::array<LighthouseSensorDirectionAngles, 4>
      deck_sensor_angles; // angles of the sensors on the deck
};

/// Generic landmark model for discrete detection sensors (both 2D and 3D).
/**
 * \note This class satisfies \ref SensorModelPage.
 *
 * \tparam StateType type of the state of the particle.
 */
class LighthouseSensorModel2D {
public:
  /// State type of a particle.
  using state_type = Sophus::SE2d;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor, detection position in robot frame
  using measurement_type = LighthouseDirectionalDetection;
  /// Map representation type.
  using map_type = LighthouseStationMapType;
  /// Parameter type
  using param_type = LighthouseSensorModelParam;

  /// Constructs a LandmarkSensorModel instance.
  explicit LighthouseSensorModel2D(param_type params,
                                   LighthouseStationMapType landmark_map)
      : params_{std::move(params)}, landmark_map_{std::move(landmark_map)} {}

  /// Returns a state weighting function conditioned on lighthouse deck
  /// measurements
  [[nodiscard]] auto operator()(measurement_type &&detections) const {
    return [this, detections = std::move(detections)](
               const state_type &state) -> weight_type {
      // The robot pose state is given in 2D. Notice that in this case
      // the 2D pose of the robot is assumed to be that of the robot footprint
      // (projection of the robot on the z=0 plane of the 3D world frame).
      // This is so that we can tie the sensor reference frame to the world
      // frame where the landmarks are given without additional structural
      // information.
      const Sophus::SE3d robot_pose_in_world = Sophus::SE3d{
          Sophus::SO3d::rotZ(state.so2().log()), //
          Eigen::Vector3d{state.translation().x(), state.translation().y(),
                          params_.sensor_height},
      };

      // if the base station is not in the map, return 1.0
      if (landmark_map_.station_poses_in_world.find(
              detections.base_station_id) ==
          landmark_map_.station_poses_in_world.end()) {
        return 1.0;
      }

      const Sophus::SE3d station_pose_in_world =
          landmark_map_.station_poses_in_world.at(detections.base_station_id);

      const auto robot_pose_in_station =
          station_pose_in_world.inverse() * robot_pose_in_world;

      const auto detection_weight = [this, &robot_pose_in_station](
                                        const auto &detection,
                                        const auto &sensor_id) {
        // calculate relative pose of the sensor in the robot frame
        const auto sensor_pose_in_robot = Sophus::SE3d{
            Sophus::SO3d{}, Eigen::Vector3d{sensor_offsets_x_[sensor_id],
                                            sensor_offsets_y_[sensor_id], 0.0}};

        // calculate observed elevation and azimuth values for the sensor
        const auto expected_sensor_direction_in_station =
            (robot_pose_in_station * sensor_pose_in_robot)
                .translation()
                .normalized();

        const auto measured_sensor_direction_in_station =
            (Sophus::SE3d{Sophus::SO3d::rotZ(detection.azimuth) *
                              Sophus::SO3d::rotY(-detection.elevation),
                          Eigen::Vector3d{0.0, 0.0, 0.0}} *
             Eigen::Vector3d{1.0, 0.0, 0.0})
                .normalized();

        // calculate the cosine similarity between the expected and measured
        // sensor directions
        const auto direction_error =
            std::acos(expected_sensor_direction_in_station.dot(
                measured_sensor_direction_in_station));

        // apply the error model
        const auto direction_error_prob = std::exp(
            -direction_error * direction_error /
            (2. * params_.sigma_angular_error * params_.sigma_angular_error));

        return direction_error_prob;
      };

      const std::array<size_t, 4> sensor_ids = {0, 1, 2, 3};
      return std::transform_reduce(detections.deck_sensor_angles.cbegin(), //
                                   detections.deck_sensor_angles.cend(),   //
                                   sensor_ids.cbegin(),                    //
                                   1.0,                                    //
                                   std::multiplies{},                      //
                                   detection_weight                        //
      );
    };
  }

  /// Update the sensor model with a new landmark `map`.
  void update_map(map_type &&map) { landmark_map_ = std::move(map); }

private:
  double sensor_x_distance_{0.0296}; ///< distance between sensors in x
  double sensor_y_distance_{0.0150}; ///< distance between sensors in y

  std::array<double, 4> sensor_offsets_x_{
      -sensor_x_distance_ / 2.0,
      -sensor_x_distance_ / 2.0,
      +sensor_x_distance_ / 2.0,
      +sensor_x_distance_ / 2.0,
  }; ///< offsets of the sensors in x
  std::array<double, 4> sensor_offsets_y_{
      +sensor_y_distance_ / 2.0,
      -sensor_y_distance_ / 2.0,
      +sensor_y_distance_ / 2.0,
      -sensor_y_distance_ / 2.0,
  }; ///< offsets of the sensors in y

  param_type params_; ///< Parameters of the sensor model.

  map_type landmark_map_; ///< Map of landmarks in the world frame.
};

} // namespace beluga

#endif // BELUGA_LIGHTHOUSE_SENSOR_MODEL_HPP
