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

#include "lighthouse_localization/lighthouse_localization_node.hpp"

namespace lighthouse_localization
{

LighthouseLocalizationNode::LighthouseLocalizationNode(const rclcpp::NodeOptions & options)
: Node("lighthouse_localization", options)
{
  // Declare and get parameter for stations map file
  declare_parameter("stations_map", "");
  std::string stations_map_path = get_parameter("stations_map").as_string();

  // Declare and get parameter for map frame
  declare_parameter("map_frame", "map");
  map_frame_ = get_parameter("map_frame").as_string();

  // Create subscription to lighthouse measurements
  subscription_ = create_subscription<lighthouse_deck_msgs::msg::LighthouseDeckMeasurement>(
    "lighthouse", rclcpp::QoS(10).best_effort(),
    [this](const lighthouse_deck_msgs::msg::LighthouseDeckMeasurement::SharedPtr msg) {
      lighthouse_callback(msg);
    });

  // Create service to set station poses
  set_station_poses_service_ = create_service<lighthouse_station_mapper_msgs::srv::SetStationPoses>(
    "set_station_poses",
    [this](
      const std::shared_ptr<lighthouse_station_mapper_msgs::srv::SetStationPoses::Request> request,
      std::shared_ptr<lighthouse_station_mapper_msgs::srv::SetStationPoses::Response> response) {
      set_station_poses_callback(request, response);
    });

  // Create publisher for deck pose
  deck_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "deck_pose", rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(), "Lighthouse localization node started");

  // Load stations map from file if path is provided
  if (!stations_map_path.empty()) {
    auto result = lighthouse_deck_utils::load_stations_map(stations_map_path);
    if (result) {
      std::tie(station_poses_, station_ids_) = std::move(*result);
      station_poses_configured_ = true;
    }
  }
}

void LighthouseLocalizationNode::lighthouse_callback(
  const lighthouse_deck_msgs::msg::LighthouseDeckMeasurement::SharedPtr msg)
{
  // Check if station poses have been configured
  if (!station_poses_configured_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Station poses not configured. Call set_station_poses service first.");
    return;
  }

  // Create optimizer with known station poses
  lighthouse_geometry_utils::DeckPoseOptimization optimizer(station_poses_, station_ids_);

  // Convert measurements to optimizer samples
  std::vector<lighthouse_geometry_utils::DeckPoseOptimization::Sample> deck_samples;

  constexpr double kDegToRad = M_PI / 180.0;
  const std::size_t n = msg->station_id.size();

  for (std::size_t i = 0; i < n; ++i) {
    lighthouse_geometry_utils::DeckPoseOptimization::Sample sample;
    sample.station_id = static_cast<lighthouse_geometry_utils::StationId>(msg->station_id[i]);

    // Convert from degrees (message) to radians (solver API)
    sample.azimuths = {
      msg->azimuth_0[i] * kDegToRad,
      msg->azimuth_1[i] * kDegToRad,
      msg->azimuth_2[i] * kDegToRad,
      msg->azimuth_3[i] * kDegToRad
    };
    sample.elevations = {
      msg->elevation_0[i] * kDegToRad,
      msg->elevation_1[i] * kDegToRad,
      msg->elevation_2[i] * kDegToRad,
      msg->elevation_3[i] * kDegToRad
    };

    deck_samples.push_back(sample);
  }

  if (deck_samples.empty()) {
    return;
  }

  try {
    // Solve for deck pose
    auto [deck_pose, deck_auto_cov] = optimizer.solve(deck_samples);

    // Publish deck pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.header.frame_id = map_frame_;

    const auto t = deck_pose.translation();
    const auto q = deck_pose.unit_quaternion();

    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    deck_pose_pub_->publish(pose_msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Failed to solve deck pose: %s", e.what());
  }
}

void LighthouseLocalizationNode::set_station_poses_callback(
  const std::shared_ptr<lighthouse_station_mapper_msgs::srv::SetStationPoses::Request> request,
  std::shared_ptr<lighthouse_station_mapper_msgs::srv::SetStationPoses::Response> response)
{
  // Clear existing data (always, regardless of request content)
  station_poses_.clear();
  station_ids_.clear();
  station_poses_configured_ = false;

  if (request->station_poses.empty()) {
    response->success = false;
    response->message = "No station poses provided";
    RCLCPP_WARN(get_logger(), "Received empty station poses");
    return;
  }

  // Convert geometry_msgs/Pose to Sophus::SE3d and store
  for (const auto & station_pose_msg : request->station_poses) {
    const auto & pose = station_pose_msg.pose;

    // Extract translation
    Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);

    // Extract rotation (quaternion in geometry_msgs is x, y, z, w)
    Eigen::Quaterniond quaternion(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z
    );

    // Create SE3 pose
    Sophus::SE3d se3_pose(quaternion, translation);

    // Store pose and ID
    station_poses_.push_back(se3_pose);
    station_ids_.push_back(
      static_cast<lighthouse_geometry_utils::StationId>(station_pose_msg.
      station_id));
  }

  station_poses_configured_ = true;

  response->success = true;
  response->message = "Successfully set " + std::to_string(station_poses_.size()) +
    " station poses";

  RCLCPP_INFO(
    get_logger(),
    "Configured %zu station poses for localization",
    station_poses_.size());
}

}  // namespace lighthouse_localization
