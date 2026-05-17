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

#include "lighthouse_station_mapper/mapper_ui_node.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <lighthouse_deck_msgs/msg/lighthouse_deck_measurement.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lighthouse_geometry_utils/deck_pose_optimization.hpp"
#include "lighthouse_geometry_utils/station_geometry_optimization.hpp"
#include "lighthouse_station_mapper/datatypes.hpp"
#include "lighthouse_station_mapper/mapper_screen_renderer.hpp"

using LighthouseDeckMeasurement = lighthouse_deck_msgs::msg::LighthouseDeckMeasurement;

namespace lighthouse_station_mapper
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kMinKeypointDistance = 0.05;  // 5cm minimum distance between keypoints
constexpr double kColinearityThreshold = 0.1;  // Threshold for colinearity check
}  // namespace

MapperUiNode::MapperUiNode(const rclcpp::NodeOptions & options)
: Node("mapper_ui", options), renderer_(std::make_unique<MapperScreenRenderer>())
{
  declare_parameter("max_angular_spread", 5e-3);
  max_angular_spread_ = get_parameter("max_angular_spread").as_double();

  declare_parameter("buffer_duration", 2.5);
  buffer_duration_ = get_parameter("buffer_duration").as_double();

  declare_parameter("min_samples_per_station", 10);
  min_samples_per_station_ = get_parameter("min_samples_per_station").as_int();

  subscription_ = create_subscription<LighthouseDeckMeasurement>(
    "lighthouse", rclcpp::QoS(10).best_effort(),
    [this](const LighthouseDeckMeasurement::SharedPtr msg) {
      lighthouse_callback(msg);
    });

  timer_ = create_wall_timer(
    std::chrono::milliseconds(100), [this] {
      timer_callback();
    });

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "lighthouse_markers", rclcpp::QoS(1).transient_local());

  deck_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "deck_pose", rclcpp::QoS(10));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  visualization_timer_ = create_wall_timer(
    std::chrono::seconds(1), [this] {
      visualization_timer_callback();
    });

  // The renderer runs its event loop on a background thread. Instead of
  // calling node methods directly (which would race with the ROS executor),
  // each callback enqueues a command using the thread-safe command queue.
  // The timer callback processes the queue on the ROS thread.
  renderer_->set_sample_callback(
    [this] {
      command_queue_.enqueue(
        [this] {
          on_sample_button_callback();
        });
    });
  renderer_->set_solve_callback(
    [this] {
      command_queue_.enqueue(
        [this] {
          on_solve_button_callback();
        });
    });
  renderer_->set_solve_keypoint_callback(
    [this] {
      command_queue_.enqueue(
        [this] {
          on_solve_keypoint_button_callback();
        });
    });
  renderer_->set_save_callback(
    [this] {
      command_queue_.enqueue(
        [this] {
          on_save_button_callback();
        });
    });
  renderer_->set_clear_samples_callback(
    [this] {
      command_queue_.enqueue(
        [this] {
          on_clear_samples_button_callback();
        });
    });
  renderer_->set_set_keypoint_callback(
    [this] {
      command_queue_.enqueue(
        [this] {
          on_set_keypoint_button_callback();
        });
    });
  renderer_->set_clear_origin_keypoints_callback(
    [this] {
      command_queue_.enqueue(
        [this] {
          on_clear_origin_keypoints_button_callback();
        });
    });
  renderer_->set_quit_callback(
    [this] {
      command_queue_.enqueue(
        [this] {
          on_quit_button_callback();
        });
    });

  renderer_->start();
}

void MapperUiNode::stop_renderer()
{
  renderer_->stop();
}

bool MapperUiNode::is_quit_requested() const
{
  return quit_requested_.load();
}

void MapperUiNode::lighthouse_callback(const LighthouseDeckMeasurement::SharedPtr msg)
{
  const rclcpp::Time timestamp{msg->header.stamp};
  const std::size_t n = msg->station_id.size();
  for (std::size_t i = 0; i < n; ++i) {
    TimestampedLighthouseSample sample;
    sample.timestamp = timestamp;
    sample.sample.station_id = static_cast<StationId>(msg->station_id[i]);
    // Convert from degrees (message) to radians (solver API)
    sample.sample.azimuth = {
      msg->azimuth_0[i] * kDegToRad, msg->azimuth_1[i] * kDegToRad,
      msg->azimuth_2[i] * kDegToRad, msg->azimuth_3[i] * kDegToRad};
    sample.sample.elevation = {
      msg->elevation_0[i] * kDegToRad, msg->elevation_1[i] * kDegToRad,
      msg->elevation_2[i] * kDegToRad, msg->elevation_3[i] * kDegToRad};
    sample_queue_.push_back(std::move(sample));
  }
  prune_old_samples();
}

void MapperUiNode::process_command_queue()
{
  prune_old_samples();

  // Process all queued commands from the UI thread
  command_queue_.process();
}

void MapperUiNode::timer_callback()
{
  prune_old_samples();

  process_command_queue();

  const auto summarized_samples = summarize_buffer();
  renderer_->set_visible_stations(summarized_samples);
  const auto [ready, message] = is_ready_to_sample(summarized_samples);
  // we do not set the message not to override other messages by the solver
  renderer_->set_sampling_status(ready);

  // if there's a solution to the geometry, use it to calculate
  // the pose of the deck and display it on the UI
  if (station_geometry_result_.has_value()) {
    const auto & geo = station_geometry_result_.value();
    lighthouse_geometry_utils::DeckPoseOptimization optimizer(
      geo.station_poses, geo.station_ids);

    std::vector<lighthouse_geometry_utils::DeckPoseOptimization::Sample> deck_samples;
    for (const auto & s : summarized_samples) {
      deck_samples.push_back(
        {s.elevation, s.azimuth, s.station_id});
    }

    if (deck_samples.empty()) {
      renderer_->clear_deck_pose();
      current_deck_pose_.reset();
    } else {
      try {
        auto [deck_pose, deck_auto_cov] = optimizer.solve(deck_samples);
        renderer_->set_deck_pose(deck_pose, deck_auto_cov);
        current_deck_pose_ = deck_pose;
        publish_deck_pose(deck_pose);
      } catch (const std::exception & e) {
        renderer_->clear_deck_pose();
        current_deck_pose_.reset();
      }
    }
  } else {
    current_deck_pose_.reset();
  }
}

void MapperUiNode::on_sample_button_callback()
{
  const auto summarized_samples = summarize_buffer();
  renderer_->set_visible_stations(summarized_samples);
  const auto [ready, message] = is_ready_to_sample(summarized_samples);
  renderer_->set_sampling_status(ready);
  renderer_->set_message(message);

  if (!ready) {
    // not ready to sample, the message explains why
    return;
  }

  const auto deck_pose_id = get_next_deck_pose_id();
  for (const auto & sample : summarized_samples) {
    RCLCPP_INFO(
      get_logger(),
      "Station %zu: count=%zu spread=%.6f° az=[%.6f°, %.6f°, %.6f°, "
      "%.6f°] el=[%.6f°, %.6f°, %.6f°, %.6f°]",
      sample.station_id, sample.count, sample.spread * kRadToDeg,
      sample.azimuth[0] * kRadToDeg, sample.azimuth[1] * kRadToDeg,
      sample.azimuth[2] * kRadToDeg, sample.azimuth[3] * kRadToDeg,
      sample.elevation[0] * kRadToDeg, sample.elevation[1] * kRadToDeg,
      sample.elevation[2] * kRadToDeg, sample.elevation[3] * kRadToDeg);
    LighthouseSample s;
    s.station_id = sample.station_id;
    s.deck_pose_id = deck_pose_id;
    s.azimuth = sample.azimuth;
    s.elevation = sample.elevation;
    samples_taken_.push_back(std::move(s));
  }

  renderer_->set_current_samples(samples_taken_);
}

void MapperUiNode::on_solve_button_callback()
{
  solve_impl(false);
}

void MapperUiNode::on_solve_keypoint_button_callback()
{
  solve_impl(true);
}

void MapperUiNode::solve_impl(bool use_keypoints)
{
  if (use_keypoints && keypoints_.size() != 3) {
    RCLCPP_WARN(
      get_logger(), "Cannot solve with keypoints: need 3 keypoints, have %zu",
      keypoints_.size());
    renderer_->set_message("[Error] Set 3 keypoints before using keypoint-based solve");
    return;
  }

  try {
    if (samples_taken_.empty()) {
      RCLCPP_WARN(get_logger(), "No samples to solve. Take samples first.");
      renderer_->set_message("[Error] No samples to solve");
      return;
    }

    const std::string origin_method = use_keypoints ? "keypoints" : "first station";
    RCLCPP_INFO(
      get_logger(), "Starting station geometry optimization with %zu samples (origin @%s)",
      samples_taken_.size(), origin_method.c_str());
    renderer_->set_message("Solving...");

    lighthouse_geometry_utils::StationGeometryOptimization optimizer;
    for (const auto & sample : samples_taken_) {
      optimizer.addSample(sample.elevation, sample.azimuth, sample.station_id, sample.deck_pose_id);
    }

    auto result = optimizer.solve();

    // Station 0 defines the raw-solver origin. When using keypoints, the
    // keypoints were recorded in F1 (where station 0 is the origin), so
    // re-anchor them back to the raw solver frame before applying the offset.
    Sophus::SE3d reference_frame = result.station_poses[0];
    if (use_keypoints) {
      reference_frame = reference_frame * compute_keypoint_origin_in_current_frame();
    }

    for (Sophus::SE3d & pose : result.station_poses) {
      pose = reference_frame.inverse() * pose;
    }

    station_geometry_result_ = result;
    renderer_->set_station_poses(result);

    RCLCPP_INFO(
      get_logger(), "Optimization complete. Found poses for %zu stations, origin set by %s",
      result.station_poses.size(), origin_method.c_str());
    renderer_->set_message("[OK] Optimization complete, origin set by " + origin_method);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Optimization failed: %s", e.what());
    renderer_->set_message(std::string("[Error] Optimization failed: ") + e.what());
  }
}

void MapperUiNode::on_save_button_callback()
{
  if (!station_geometry_result_.has_value()) {
    RCLCPP_WARN(get_logger(), "No solution to save");
    renderer_->set_message("[Error] No solution to save — solve first");
    return;
  }

  // Build a timestamped filename in the current working directory.
  const auto now = std::chrono::system_clock::now();
  const auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  localtime_r(&t, &tm_buf);
  std::ostringstream filename_oss;
  filename_oss << "lighthouse_map_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".csv";
  const std::string filename = filename_oss.str();

  std::ofstream file(filename);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open file for writing: %s", filename.c_str());
    renderer_->set_message("[Error] Could not open file: " + filename);
    return;
  }

  // Header
  file << "station_id,x,y,z,qx,qy,qz,qw\n";
  file << std::fixed << std::setprecision(9);

  const auto & result = station_geometry_result_.value();
  for (std::size_t i = 0; i < result.station_ids.size(); ++i) {
    const auto & pose = result.station_poses[i];
    const auto t_vec = pose.translation();
    const auto q = pose.unit_quaternion();
    file
      << result.station_ids[i] << ","
      << t_vec.x() << "," << t_vec.y() << "," << t_vec.z() << ","
      << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
  }

  if (!file) {
    RCLCPP_ERROR(get_logger(), "Error while writing file: %s", filename.c_str());
    renderer_->set_message("[Error] Write error on file: " + filename);
    return;
  }

  RCLCPP_INFO(get_logger(), "Station map saved to %s", filename.c_str());
  renderer_->set_message("[OK] Map saved to " + filename);
}

void MapperUiNode::on_set_keypoint_button_callback()
{
  // Check if we have a valid deck pose
  if (!current_deck_pose_.has_value()) {
    renderer_->set_message("[Error] No deck pose available");
    return;
  }

  // Check if we already have 3 keypoints
  if (keypoints_.size() >= 3) {
    renderer_->set_message("[Error] Too many keypoints, keypoint ignored");
    return;
  }

  const auto & new_pose = current_deck_pose_.value();
  const auto new_translation = new_pose.translation();

  // Check if the new keypoint is too close to existing ones
  for (size_t i = 0; i < keypoints_.size(); ++i) {
    const auto & existing_translation = keypoints_[i].translation();
    const double distance = (new_translation - existing_translation).norm();
    if (distance < kMinKeypointDistance) {
      renderer_->set_message(
        "[Error] Keypoint too close to keypoint " + std::to_string(i) +
        " (distance: " + std::to_string(distance * 100.0) + " cm)");
      return;
    }
  }

  // If adding a third keypoint, check for colinearity with first two
  if (keypoints_.size() == 2) {
    const auto v01 = (keypoints_[1].translation() - keypoints_[0].translation()).normalized();
    const auto v02 = (new_translation - keypoints_[0].translation()).normalized();
    const double cross_product_norm = v01.cross(v02).norm();
    if (cross_product_norm < kColinearityThreshold) {
      renderer_->set_message(
        "[Error] Keypoints are colinear (cross product: " +
        std::to_string(cross_product_norm) + ")");
      return;
    }
  }

  // Add the keypoint
  keypoints_.push_back(new_pose);

  // Set success message based on which keypoint was added
  std::string message;
  switch (keypoints_.size()) {
    case 1:
      message = "[OK] Origin keypoint set";
      break;
    case 2:
      message = "[OK] X direction keypoint set";
      break;
    case 3:
      message = "[OK] XY plane keypoint set — ready to solve with keypoints";
      break;
    default:
      message = "[OK] Keypoint added";
      break;
  }

  RCLCPP_INFO(
    get_logger(), "Keypoint %zu added at position [%.3f, %.3f, %.3f]",
    keypoints_.size(),
    new_translation.x(), new_translation.y(), new_translation.z());
  renderer_->set_message(message);
}

void MapperUiNode::on_clear_origin_keypoints_button_callback()
{
  keypoints_.clear();
  RCLCPP_INFO(get_logger(), "Origin keypoints cleared");
  renderer_->set_message("[OK] Origin keypoints cleared");
}

Sophus::SE3d MapperUiNode::compute_keypoint_origin_in_current_frame() const
{
  // This function should only be called when 3 keypoints are set
  if (keypoints_.size() != 3) {
    RCLCPP_ERROR(
      get_logger(),
      "compute_keypoint_origin_in_current_frame called without 3 keypoints (have %zu)",
      keypoints_.size());
    throw std::runtime_error("Not enough keypoints to determine the origin frame");
  }

  const Eigen::Vector3d & origin = keypoints_[0].translation();
  const Eigen::Vector3d & point_x = keypoints_[1].translation();
  const Eigen::Vector3d & point_plane = keypoints_[2].translation();

  // X axis points from origin to second keypoint
  const Eigen::Vector3d x_axis = (point_x - origin).normalized();

  // Z axis is perpendicular to the plane defined by the 3 points
  const Eigen::Vector3d v1 = point_x - origin;
  const Eigen::Vector3d v2 = point_plane - origin;
  const Eigen::Vector3d z_axis = v1.cross(v2).normalized();

  // Y axis completes the right-handed frame
  const Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

  // Build rotation matrix from axes
  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;

  // Create SE3 transformation
  Sophus::SE3d new_origin_in_current_frame(rotation, origin);

  return new_origin_in_current_frame;
}

void MapperUiNode::on_clear_samples_button_callback()
{
  sample_queue_.clear();
  samples_taken_.clear();
  station_geometry_result_.reset();
  current_deck_pose_.reset();
  keypoints_.clear();
  next_deck_pose_id_ = 0;
  renderer_->set_current_samples(samples_taken_);
  renderer_->set_station_poses({});
  renderer_->set_message("[OK] Samples cleared");
  renderer_->clear_deck_pose();
}

void MapperUiNode::prune_old_samples()
{
  const rclcpp::Time cutoff = now() - rclcpp::Duration::from_seconds(buffer_duration_);
  while (!sample_queue_.empty() && sample_queue_.front().timestamp < cutoff) {
    sample_queue_.pop_front();
  }
}

DeckPoseId MapperUiNode::get_next_deck_pose_id()
{
  return next_deck_pose_id_++;
}

std::vector<SummarizedStationData> MapperUiNode::summarize_buffer()
{
  prune_old_samples();

  // Group buffered samples by station id.
  std::map<StationId, AccumulatedSensorReadings> by_station;

  // classify samples by station_id
  for (const auto & ts : sample_queue_) {
    auto & r = by_station[ts.sample.station_id];
    r.sensor_0_azimuth.push_back(ts.sample.azimuth[0]);
    r.sensor_0_elevation.push_back(ts.sample.elevation[0]);
    r.sensor_1_azimuth.push_back(ts.sample.azimuth[1]);
    r.sensor_1_elevation.push_back(ts.sample.elevation[1]);
    r.sensor_2_azimuth.push_back(ts.sample.azimuth[2]);
    r.sensor_2_elevation.push_back(ts.sample.elevation[2]);
    r.sensor_3_azimuth.push_back(ts.sample.azimuth[3]);
    r.sensor_3_elevation.push_back(ts.sample.elevation[3]);
  }

  auto stats_of = [](std::vector<double> & v) -> std::tuple<double, double, double> {
      if (v.empty()) {
        return std::tuple{0.0, 0.0, 0.0};
      }
      auto max = std::max_element(v.begin(), v.end());
      auto min = std::min_element(v.begin(), v.end());
      auto value = v.back();
      return std::tuple{*min, *max, value};
    };

  std::vector<SummarizedStationData> result;
  for (auto & [station_id, readings] : by_station) {
    // check if we meet the minimum sample count requirement for this station
    const int count = static_cast<int>(readings.sensor_0_azimuth.size());

    const auto [s0_az_min, s0_az_max, s0_az_latest] = stats_of(readings.sensor_0_azimuth);
    const auto [s0_el_min, s0_el_max, s0_el_latest] = stats_of(readings.sensor_0_elevation);
    const auto [s1_az_min, s1_az_max, s1_az_latest] = stats_of(readings.sensor_1_azimuth);
    const auto [s1_el_min, s1_el_max, s1_el_latest] = stats_of(readings.sensor_1_elevation);
    const auto [s2_az_min, s2_az_max, s2_az_latest] = stats_of(readings.sensor_2_azimuth);
    const auto [s2_el_min, s2_el_max, s2_el_latest] = stats_of(readings.sensor_2_elevation);
    const auto [s3_az_min, s3_az_max, s3_az_latest] = stats_of(readings.sensor_3_azimuth);
    const auto [s3_el_min, s3_el_max, s3_el_latest] = stats_of(readings.sensor_3_elevation);

    const auto s0_az_spread = s0_az_max - s0_az_min;
    const auto s0_el_spread = s0_el_max - s0_el_min;
    const auto s1_az_spread = s1_az_max - s1_az_min;
    const auto s1_el_spread = s1_el_max - s1_el_min;
    const auto s2_az_spread = s2_az_max - s2_az_min;
    const auto s2_el_spread = s2_el_max - s2_el_min;
    const auto s3_az_spread = s3_az_max - s3_az_min;
    const auto s3_el_spread = s3_el_max - s3_el_min;

    const auto max_spread = std::max(
      {s0_az_spread, s0_el_spread, s1_az_spread, s1_el_spread, s2_az_spread, s2_el_spread,
        s3_az_spread, s3_el_spread});

    SummarizedStationData sample;
    sample.station_id = station_id;
    sample.azimuth = {s0_az_latest, s1_az_latest, s2_az_latest, s3_az_latest};
    sample.elevation = {s0_el_latest, s1_el_latest, s2_el_latest, s3_el_latest};
    sample.count = count;
    sample.spread = max_spread;
    result.push_back(std::move(sample));
  }

  return result;
}

std::tuple<bool, std::string> MapperUiNode::is_ready_to_sample(
  const std::vector<SummarizedStationData> & samples)
{
  if (samples.empty()) {
    const std::string msg = "[Error] Sampling failed: no stations in buffer.";
    return std::make_tuple(false, msg);
  }

  for (const auto & sample : samples) {
    if (sample.count < static_cast<std::size_t>(min_samples_per_station_)) {
      const std::string msg = "[Error] Sampling failed: station " +
        std::to_string(sample.station_id) +
        " has only " + std::to_string(sample.count) + " samples (min " +
        std::to_string(min_samples_per_station_) + " required).";
      return std::make_tuple(false, msg);
    }
    if (sample.spread > max_angular_spread_) {
      const std::string msg = "[Error] Sampling failed: station " +
        std::to_string(sample.station_id) +
        " angular spread " + std::to_string(sample.spread) +
        " exceeds tolerance " + std::to_string(max_angular_spread_) + ".";
      return std::make_tuple(false, msg);
    }
  }

  return std::make_tuple(true, "[OK] Sample taken!");
}

void MapperUiNode::on_quit_button_callback()
{
  quit_requested_.store(true);
}

void MapperUiNode::visualization_timer_callback()
{
  if (!station_geometry_result_.has_value()) {
    return;
  }

  const auto & geo = station_geometry_result_.value();
  lighthouse_geometry_utils::DeckPoseOptimization optimizer(geo.station_poses, geo.station_ids);

  // Group stored samples by deck pose id and solve for each position.
  std::map<DeckPoseId, std::vector<LighthouseSample>> by_pose_id;
  for (const auto & s : samples_taken_) {
    by_pose_id[s.deck_pose_id].push_back(s);
  }

  std::vector<Sophus::SE3d> sample_poses;
  for (const auto & [pose_id, pose_samples] : by_pose_id) {
    std::vector<lighthouse_geometry_utils::DeckPoseOptimization::Sample> deck_samples;
    for (const auto & s : pose_samples) {
      deck_samples.push_back({s.elevation, s.azimuth, s.station_id});
    }
    try {
      if (deck_samples.size() > 1) {
        deck_samples.resize(1);
      }
      auto [pose, cov] = optimizer.solve(deck_samples);
      sample_poses.push_back(pose);
    } catch (const std::exception &) {
      // Skip positions that cannot be solved.
    }
  }

  // Keypoints are recorded in F1 (station 0 at origin). When a keypoint-based
  // solve has been done, the world frame is the keypoint-origin frame, so
  // transform the keypoints by the same reference_frame.inverse() that was
  // applied to the station poses (= compute_keypoint_origin_in_current_frame().inverse()).
  std::vector<Sophus::SE3d> transformed_keypoints;
  if (keypoints_.size() == 3) {
    const auto origin = compute_keypoint_origin_in_current_frame();
    for (const auto & kp : keypoints_) {
      transformed_keypoints.push_back(origin.inverse() * kp);
    }
  } else {
    transformed_keypoints = keypoints_;
  }

  publish_markers(geo.station_poses, sample_poses, transformed_keypoints);
  publish_transforms(geo.station_poses, sample_poses, transformed_keypoints);
}

namespace
{
/// Build a geometry_msgs Pose from a Sophus SE3d.
geometry_msgs::msg::Pose se3_to_pose_msg(const Sophus::SE3d & pose)
{
  geometry_msgs::msg::Pose msg;
  const auto t = pose.translation();
  const auto q = pose.unit_quaternion();
  msg.position.x = t.x();
  msg.position.y = t.y();
  msg.position.z = t.z();
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

/// Append a sphere marker to @p markers.
void add_sphere(
  visualization_msgs::msg::MarkerArray & markers,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & ns,
  int id,
  const Sophus::SE3d & pose,
  float r, float g, float b,
  double radius = 0.1)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose = se3_to_pose_msg(pose);
  m.scale.x = m.scale.y = m.scale.z = radius;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 1.0f;
  markers.markers.push_back(std::move(m));
}

}  // namespace

void MapperUiNode::publish_markers(
  const std::vector<Sophus::SE3d> & station_poses,
  const std::vector<Sophus::SE3d> & sample_poses,
  const std::vector<Sophus::SE3d> & keypoints)
{
  visualization_msgs::msg::MarkerArray markers;
  const auto stamp = now();
  constexpr char kFrame[] = "map";

  // Delete all previous markers first.
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(del);

  // Stations — white sphere.
  for (std::size_t i = 0; i < station_poses.size(); ++i) {
    add_sphere(
      markers, kFrame, stamp, "stations", static_cast<int>(i), station_poses[i], 1, 1, 1,
      0.15);
  }

  // Samples — green sphere.
  for (std::size_t i = 0; i < sample_poses.size(); ++i) {
    add_sphere(
      markers, kFrame, stamp, "samples", static_cast<int>(i), sample_poses[i], 0, 0.8f, 0, 0.15);
  }

  // Keypoints — yellow sphere only.
  for (std::size_t i = 0; i < keypoints.size(); ++i) {
    add_sphere(
      markers, kFrame, stamp, "keypoints", static_cast<int>(i), keypoints[i], 1, 1, 0, 0.15);
  }

  marker_pub_->publish(markers);
}

void MapperUiNode::publish_deck_pose(const Sophus::SE3d & pose)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.pose = se3_to_pose_msg(pose);
  deck_pose_pub_->publish(msg);
}

void MapperUiNode::publish_transforms(
  const std::vector<Sophus::SE3d> & station_poses,
  const std::vector<Sophus::SE3d> & sample_poses,
  const std::vector<Sophus::SE3d> & keypoints)
{
  const auto stamp = now();
  std::vector<geometry_msgs::msg::TransformStamped> transforms;

  auto make_tf = [&stamp](const std::string & child_frame, const Sophus::SE3d & pose)
    -> geometry_msgs::msg::TransformStamped {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = "map";
      tf.child_frame_id = child_frame;
      const auto t = pose.translation();
      const auto q = pose.unit_quaternion();
      tf.transform.translation.x = t.x();
      tf.transform.translation.y = t.y();
      tf.transform.translation.z = t.z();
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      return tf;
    };

  for (std::size_t i = 0; i < station_poses.size(); ++i) {
    transforms.push_back(make_tf("station_" + std::to_string(i), station_poses[i]));
  }

  for (std::size_t i = 0; i < sample_poses.size(); ++i) {
    transforms.push_back(make_tf("sample_" + std::to_string(i), sample_poses[i]));
  }

  for (std::size_t i = 0; i < keypoints.size(); ++i) {
    transforms.push_back(make_tf("keypoint_" + std::to_string(i), keypoints[i]));
  }

  tf_broadcaster_->sendTransform(transforms);
}

}  // namespace lighthouse_station_mapper
