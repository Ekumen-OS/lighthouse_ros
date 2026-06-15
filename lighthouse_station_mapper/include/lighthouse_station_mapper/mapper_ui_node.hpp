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

#ifndef LIGHTHOUSE_STATION_MAPPER__MAPPER_UI_NODE_HPP_
#define LIGHTHOUSE_STATION_MAPPER__MAPPER_UI_NODE_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include <lighthouse_deck_msgs/msg/lighthouse_deck_measurement.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lighthouse_geometry_utils/station_geometry_optimization.hpp"
#include "lighthouse_station_mapper/command_queue.hpp"
#include "lighthouse_station_mapper/datatypes.hpp"
#include "lighthouse_station_mapper/mapper_screen_renderer.hpp"

namespace lighthouse_station_mapper
{

/**
 * @brief ROS 2 node that provides a terminal UI for interactively mapping lighthouse base-station geometry.
 *
 * This node subscribes to raw lighthouse measurements, buffers them in a sliding time window,
 * and provides an FTXUI-based terminal interface for interactive station mapping. Users can
 * take samples at different deck positions, solve for station poses using optimization,
 * visualize results in RViz, and save the calibrated geometry.
 *
 * The node manages communication between the ROS 2 executor thread and the FTXUI renderer
 * thread using a command queue pattern.
 */
class MapperUiNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new MapperUiNode.
   *
   * Initializes the node, loads parameters, creates subscriptions and publishers,
   * sets up timers, and starts the FTXUI renderer.
   *
   * @param options ROS 2 node options for configuration.
   */
  explicit MapperUiNode(const rclcpp::NodeOptions & options);

  /**
   * @brief Stop the FTXUI screen renderer.
   *
   * Must be called before destruction to cleanly join the renderer background thread.
   * Safe to call multiple times.
   */
  void stop_renderer();

  /**
   * @brief Check if the user has pressed the Quit button.
   *
   * Thread-safe atomic read.
   *
   * @return True if quit was requested, false otherwise.
   */
  bool is_quit_requested() const;

private:
  /// Per-station accumulator for raw sensor readings, used internally
  /// by summarize_buffer() to collect values before computing medians.
  struct AccumulatedSensorReadings
  {
    /// Azimuth readings from sensor 0 (radians).
    std::vector<double> sensor_0_azimuth;
    /// Elevation readings from sensor 0 (radians).
    std::vector<double> sensor_0_elevation;
    /// Azimuth readings from sensor 1 (radians).
    std::vector<double> sensor_1_azimuth;
    /// Elevation readings from sensor 1 (radians).
    std::vector<double> sensor_1_elevation;
    /// Azimuth readings from sensor 2 (radians).
    std::vector<double> sensor_2_azimuth;
    /// Elevation readings from sensor 2 (radians).
    std::vector<double> sensor_2_elevation;
    /// Azimuth readings from sensor 3 (radians).
    std::vector<double> sensor_3_azimuth;
    /// Elevation readings from sensor 3 (radians).
    std::vector<double> sensor_3_elevation;
    /// Timestamp of the latest sample (ROS time).
    rclcpp::Time latest_timestamp;
  };

  /// A single raw lighthouse measurement tagged with its receive time,
  /// used for buffering and age-based pruning.
  struct TimestampedLighthouseSample
  {
    /// Time when this sample was received.
    rclcpp::Time timestamp;
    /// The summarized station measurement data.
    SummarizedStationData sample;
  };

  /**
   * @brief Subscription callback. Pushes incoming measurements into
   * sample_queue_ and prunes old entries.
   * @param msg Incoming lighthouse deck measurement message.
   */
  void lighthouse_callback(
    const lighthouse_deck_msgs::msg::LighthouseDeckMeasurement::SharedPtr msg);

  /**
   * @brief Timer callback that drains command_queue_ and refreshes the UI.
   *
   * Runs on the ROS executor thread. Processes all queued commands from the renderer
   * thread, then updates the visible-stations table in the UI.
   */
  void timer_callback();

  /**
   * @brief Process the command queue and update UI state.
   *
   * Drains command_queue_, processes pending commands, summarizes the buffer,
   * updates UI state, and solves deck pose if geometry is available.
   * This is the main processing function called by timer_callback().
   */
  void process_command_queue();

  /**
   * @brief Visualization timer callback (1 Hz).
   *
   * Re-solves deck poses for all stored samples using the latest station geometry,
   * then broadcasts visualization markers and TF transforms for RViz.
   */
  void visualization_timer_callback();

  /**
   * @brief Handler for the Sample button.
   *
   * Summarizes the current buffer, assigns a common deck_pose_id to all visible stations,
   * and appends them to samples_taken_. Only proceeds if sampling conditions are met.
   */
  void on_sample_button_callback();

  /**
   * @brief Handler for the Solve button.
   *
   * Runs StationGeometryOptimization on samples_taken_ and pushes results to the UI.
   * Uses the first station pose as the origin.
   */
  void on_solve_button_callback();

  /**
   * @brief Handler for the Solve (origin @keypoint) button.
   *
   * Runs StationGeometryOptimization on samples_taken_ using keypoints as reference frame,
   * and pushes results to the UI. Requires exactly 3 keypoints to be set.
   */
  void on_solve_keypoint_button_callback();

  /**
   * @brief Shared implementation for both solve callbacks.
   * @param use_keypoints If true, requires 3 keypoints and uses them to define the
   *        origin; if false, uses the first station pose as the origin.
   */
  void solve_impl(bool use_keypoints);

  /**
   * @brief Handler for the Save button.
   *
   * Persists optimization results to disk. Not yet implemented.
   */
  void on_save_button_callback();

  /**
   * @brief Handler for the Clear Samples button.
   *
   * Resets all collected data: sample_queue_, samples_taken_, station_geometry_result_,
   * and next_deck_pose_id_.
   */
  void on_clear_samples_button_callback();

  /**
   * @brief Handler for the Set keypoint button.
   *
   * Records the current deck pose as a keypoint for coordinate frame definition.
   * Allows up to 3 keypoints to be stored.
   */
  void on_set_keypoint_button_callback();

  /**
   * @brief Handler for the Clear origin keypoints button.
   *
   * Clears all stored keypoints.
   */
  void on_clear_origin_keypoints_button_callback();

  /**
   * @brief Handler for the Quit button.
   *
   * Sets quit_requested_ to true, signaling the application to terminate.
   */
  void on_quit_button_callback();

  /**
   * @brief Publish a MarkerArray visualizing station poses, sample poses, and keypoints.
   * Stations are rendered as a white sphere with an RGB triad.
   * Samples are rendered as green spheres with an RGB triad.
   * Keypoints are rendered as yellow spheres.
   * @param station_poses Station SE3 poses (from the last solve result).
   * @param sample_poses  One representative SE3 pose per sample position.
   * @param keypoints     Stored origin keypoints.
   */
  void publish_markers(
    const std::vector<Sophus::SE3d> & station_poses,
    const std::vector<Sophus::SE3d> & sample_poses,
    const std::vector<Sophus::SE3d> & keypoints);

  /**
   * @brief Publish the current deck pose as a geometry_msgs/PoseStamped.
   * @param pose Current deck SE3 pose.
   */
  void publish_deck_pose(const Sophus::SE3d & pose);

  /**
   * @brief Broadcast TF transforms for station poses, sample poses, and keypoints.
   * Uses the same set of elements as publish_markers().
   * @param station_poses Station SE3 poses (from the last solve result).
   * @param sample_poses  One representative SE3 pose per sample position.
   * @param keypoints     Stored origin keypoints.
   */
  void publish_transforms(
    const std::vector<Sophus::SE3d> & station_poses,
    const std::vector<Sophus::SE3d> & sample_poses,
    const std::vector<Sophus::SE3d> & keypoints);

  /**
   * @brief Compute reference frame transformation from keypoints.
   *
   * If 3 keypoints are set: origin at keypoint[0], X axis toward keypoint[1],
   * Z axis perpendicular to the plane defined by the 3 points.
   * If no keypoints: uses first station pose as origin.
   *
   * @return T_world_ref: pose of the reference frame in world coordinates.
   */
  Sophus::SE3d compute_keypoint_origin_in_current_frame() const;

  /**
   * @brief Remove entries from sample_queue_ older than buffer_duration_.
   */
  void prune_old_samples();

  /**
   * @brief Consume the current sample buffer and return one
   * SummarizedStationData per station, built from the median
   * azimuth/elevation values across all buffered readings.
   * @return Vector of summarized station data, one per visible station.
   */
  std::vector<SummarizedStationData> summarize_buffer();

  /**
   * @brief Check whether the summarized buffer is ready for sampling.
   * Returns a (ready, message) tuple: ready is true when all stations
   * meet the minimum sample count and angular spread requirements;
   * message explains the result.
   * @param samples Summarized station data to check.
   * @return Tuple of (ready flag, status message).
   */
  std::tuple<bool, std::string> is_ready_to_sample(
    const std::vector<SummarizedStationData> & samples);

  /**
   * @brief Generates and returns a unique deck pose ID.
   * @return Unique deck pose identifier.
   */
  DeckPoseId get_next_deck_pose_id();

  /// Thread-safe command queue for communication between UI thread and ROS thread.
  CommandQueue command_queue_;

  /// Maximum allowed max-minus-min angular spread (radians) for a
  /// station to be considered stable enough for sampling.
  double max_angular_spread_;

  /// Duration in seconds of the sliding window for raw measurements.
  double buffer_duration_;

  /// Minimum number of raw measurements required per station before
  /// a sample can be taken.
  int min_samples_per_station_;

  /// Sliding-window buffer of timestamped raw measurements.
  std::deque<TimestampedLighthouseSample> sample_queue_;

  /// Accumulated samples the user has committed via the Sample button.
  std::vector<LighthouseSample> samples_taken_;

  /// Last successful optimization result (solved station poses).
  std::optional<lighthouse_geometry_utils::StationPoseEstimates>
  station_geometry_result_;

  /// Current deck pose estimate (if available).
  std::optional<Sophus::SE3d> current_deck_pose_;

  /// Stored keypoints for defining coordinate frame (0-3 poses).
  std::vector<Sophus::SE3d> keypoints_;

  /// Monotonically increasing counter for deck pose IDs.
  DeckPoseId next_deck_pose_id_{0};

  /// Flag set by the Quit button callback.
  std::atomic<bool> quit_requested_{false};

  /// FTXUI screen renderer running on a background thread.
  std::unique_ptr<MapperScreenRenderer> renderer_;

  /// Subscription to raw lighthouse deck measurements.
  rclcpp::Subscription<lighthouse_deck_msgs::msg::LighthouseDeckMeasurement>::SharedPtr
    subscription_;

  /// Publisher for RViz MarkerArray (stations).
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr station_markers_pub_;

  /// Publisher for RViz MarkerArray (deck pose samples).
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr deck_pose_markers_pub_;

  /// Publisher for RViz MarkerArray (keypoints).
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keypoint_markers_pub_;

  /// Publisher for the current deck pose.
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr deck_pose_pub_;

  /// TF transform broadcaster for station, sample, and keypoint frames.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /// Periodic timer that drains the command queue and refreshes the UI.
  rclcpp::TimerBase::SharedPtr timer_;

  /// 1 Hz timer that re-solves sample poses and publishes visualization.
  rclcpp::TimerBase::SharedPtr visualization_timer_;
};

}  // namespace lighthouse_station_mapper

#endif  // LIGHTHOUSE_STATION_MAPPER__MAPPER_UI_NODE_HPP_
