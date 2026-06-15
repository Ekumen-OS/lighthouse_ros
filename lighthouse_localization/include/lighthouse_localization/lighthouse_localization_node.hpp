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

#ifndef LIGHTHOUSE_LOCALIZATION__LIGHTHOUSE_LOCALIZATION_NODE_HPP_
#define LIGHTHOUSE_LOCALIZATION__LIGHTHOUSE_LOCALIZATION_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sophus/se3.hpp>
#include <lighthouse_deck_msgs/msg/lighthouse_deck_measurement.hpp>
#include <lighthouse_station_mapper_msgs/srv/set_station_poses.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "lighthouse_geometry_utils/deck_pose_optimization.hpp"
#include "lighthouse_geometry_utils/datatypes.hpp"
#include "lighthouse_deck_utils/utils.hpp"

namespace lighthouse_localization
{

/**
 * @brief ROS 2 node for lighthouse-based localization.
 *
 * This node subscribes to lighthouse measurements and provides
 * a service to set the base station poses for localization.
 */
class LighthouseLocalizationNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new LighthouseLocalizationNode.
   *
   * @param options ROS 2 node options for configuration.
   */
  explicit LighthouseLocalizationNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief Subscription callback for lighthouse measurements.
   * @param msg Incoming lighthouse deck measurement message.
   */
  void lighthouse_callback(
    const lighthouse_deck_msgs::msg::LighthouseDeckMeasurement::SharedPtr msg);

  /**
   * @brief Service callback to set station poses.
   * @param request Service request containing station poses.
   * @param response Service response with success status.
   */
  void set_station_poses_callback(
    const std::shared_ptr<lighthouse_station_mapper_msgs::srv::SetStationPoses::Request> request,
    std::shared_ptr<lighthouse_station_mapper_msgs::srv::SetStationPoses::Response> response);

  /// Subscription to raw lighthouse deck measurements.
  rclcpp::Subscription<lighthouse_deck_msgs::msg::LighthouseDeckMeasurement>::SharedPtr
    subscription_;

  /// Service to set base station poses.
  rclcpp::Service<lighthouse_station_mapper_msgs::srv::SetStationPoses>::SharedPtr
    set_station_poses_service_;

  /// Publisher for the current deck pose.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr deck_pose_pub_;

  /// Base station poses for optimization (SE3 transformations).
  std::vector<Sophus::SE3d> station_poses_;

  /// Station IDs corresponding to station_poses_.
  std::vector<lighthouse_geometry_utils::StationId> station_ids_;

  /// Flag indicating whether station poses have been configured.
  bool station_poses_configured_{false};
};

}  // namespace lighthouse_localization

#endif  // LIGHTHOUSE_LOCALIZATION__LIGHTHOUSE_LOCALIZATION_NODE_HPP_
