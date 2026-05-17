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

#ifndef LIGHTHOUSE_STATION_MAPPER__MAPPER_SCREEN_RENDERER_HPP_
#define LIGHTHOUSE_STATION_MAPPER__MAPPER_SCREEN_RENDERER_HPP_

#include <chrono>
#include <ctime>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>

#include "lighthouse_geometry_utils/datatypes.hpp"
#include "lighthouse_station_mapper/datatypes.hpp"
#include <sophus/se3.hpp>

namespace lighthouse_station_mapper
{

/**
 * @brief Main runner of the terminal-based user interface for the lighthouse station mapper.
 *
 * This class manages the interactive UI built on FTXUI.
 * It owns and controls the background thread that drives user interactions and screen rendering.
 * The UI displays real-time information about visible stations, collected samples, optimization
 * solutions, and deck pose estimates, while providing interactive buttons for controlling the
 * mapping workflow.
 *
 * Communication with the ROS application is achieved through callback functions that can be
 * registered for each interactive button (Sample, Solve, Save, etc.). When a user presses a
 * button in the UI, the corresponding callback is invoked, allowing the ROS node to respond
 * to user actions. Data flows from ROS to the UI through thread-safe setter methods
 * (set_visible_stations(), set_current_samples(), etc.).
 *
 * All public setter methods are thread-safe and can be called from ROS callbacks.
 * The internal data is protected by a mutex to prevent race conditions between the UI render
 * thread and ROS callback threads.
 */
class MapperScreenRenderer
{
public:
  /**
   * @brief Construct a new MapperScreenRenderer.
   *
   * Initializes the FTXUI screen and interactive components. The screen loop
   * is not started until start() is called.
   */
  MapperScreenRenderer();

  /**
   * @brief Destroy the MapperScreenRenderer.
   *
   * Automatically stops the screen loop if it is running and joins the background thread.
   */
  ~MapperScreenRenderer();

  /// Non-copyable, non-movable (screen and thread cannot be relocated).
  MapperScreenRenderer(const MapperScreenRenderer &) = delete;
  MapperScreenRenderer & operator=(const MapperScreenRenderer &) = delete;

  /**
   * @brief Start the interactive screen loop in a background thread.
   *
   * Launches the FTXUI event loop in a dedicated thread. This method returns immediately
   * after starting the thread. Thread-safe.
   */
  void start();

  /**
   * @brief Stop the interactive screen loop and join the background thread.
   *
   * Gracefully terminates the FTXUI event loop and waits for the thread to finish.
   * Safe to call if start() was never called, or after stop() already ran. Thread-safe.
   */
  void stop();

  /**
   * @brief Convert visible stations to table rows and push them to the renderer.
   * Thread-safe.
   * @param stations Vector of summarized station data to display.
   */
  void set_visible_stations(const std::vector<SummarizedStationData> & stations);

  /**
   * @brief Convert samples to table rows and push them to the renderer.
   * Thread-safe.
   * @param samples Vector of lighthouse samples to display.
   */
  void set_current_samples(const std::vector<LighthouseSample> & samples);

  /**
   * @brief Convert station geometry result to table rows and push them to the renderer.
   * Thread-safe.
   * @param result Station pose estimates from optimization.
   */
  void set_station_poses(const lighthouse_geometry_utils::StationPoseEstimates & result);

  /**
   * @brief Set the callback invoked when the Sample button is pressed.
   * Defaults to a no-op.
   * @param cb Callback function to invoke.
   */
  void set_sample_callback(std::function<void()> cb);

  /**
   * @brief Set the callback invoked when the Solve button is pressed.
   * Defaults to a no-op.
   * @param cb Callback function to invoke.
   */
  void set_solve_callback(std::function<void()> cb);

  /**
   * @brief Set the callback invoked when the Solve (origin @keypoint) button is pressed.
   * Defaults to a no-op.
   * @param cb Callback function to invoke.
   */
  void set_solve_keypoint_callback(std::function<void()> cb);

  /**
   * @brief Set the callback invoked when the Save button is pressed.
   * Defaults to a no-op.
   * @param cb Callback function to invoke.
   */
  void set_save_callback(std::function<void()> cb);

  /**
   * @brief Set the callback invoked when the Clear Samples button is pressed.
   * Defaults to a no-op.
   * @param cb Callback function to invoke.
   */
  void set_clear_samples_callback(std::function<void()> cb);

  /**
   * @brief Set the callback invoked when the Set keypoint button is pressed.
   * Defaults to a no-op.
   * @param cb Callback function to invoke.
   */
  void set_set_keypoint_callback(std::function<void()> cb);

  /**
   * @brief Set the callback invoked when the Clear origin keypoints button is pressed.
   * Defaults to a no-op.
   * @param cb Callback function to invoke.
   */
  void set_clear_origin_keypoints_callback(std::function<void()> cb);

  /**
   * @brief Set the callback invoked when the Quit button is pressed.
   * Defaults to a no-op.
   * @param cb Callback function to invoke.
   */
  void set_quit_callback(std::function<void()> cb);

  /**
   * @brief Update the sampling-active indicator and trigger a redraw.
   * Thread-safe.
   * @param active True if sampling is ready, false otherwise.
   */
  void set_sampling_status(bool active);

  /**
   * @brief Update the status message line and trigger a redraw.
   * Thread-safe.
   * @param message Status message to display.
   */
  void set_message(std::string message);

  /**
   * @brief Set the deck pose and its autocovariance diagonal, trigger a redraw.
   * Thread-safe.
   * @param pose Deck SE3 pose estimate.
   * @param auto_cov Autocovariance diagonal for the pose.
   */
  void set_deck_pose(
    const Sophus::SE3d & pose,
    const lighthouse_geometry_utils::AutoCovDiagonal & auto_cov);

  /**
   * @brief Clear the deck pose display and trigger a redraw.
   *
   * Resets the deck pose and autocovariance to unset state. Thread-safe.
   */
  void clear_deck_pose();

  /**
   * @brief Append a timestamped message to the log window and trigger a redraw.
   * Thread-safe. Older entries are discarded once the log exceeds 1000 lines.
   * @param message Message text to append.
   */
  void append_log(std::string message);

private:
  /**
   * @brief Append entry to log_entries_, evicting the oldest if over the limit.
   *
   * This is an internal helper that does not acquire the mutex. Caller must hold data_mutex_.
   *
   * @param entry Timestamped log entry to append.
   */
  void append_log_locked(std::string entry);

  /**
   * @brief Renders a table with the given rows and title.
   * @param rows Vector of rows, each row is a vector of cell strings.
   * @param title Title to display above the table.
   * @return FTXUI element representing the rendered table.
   */
  static ftxui::Element render_table(
    const std::vector<std::vector<std::string>> & rows, const std::string & title);

  /**
   * @brief Builds the main UI layout element.
   * @param visible_stations_rows Table rows for visible stations.
   * @param samples_rows Table rows for collected samples.
   * @param solution_rows Table rows for optimization solution.
   * @param button_column FTXUI element containing the button bar.
   * @param deck_pose Optional deck pose to display.
   * @param deck_auto_cov Optional deck autocovariance to display.
   * @param sampling_active True if sampling is ready.
   * @param message Status message to display.
   * @param log_entries Timestamped log entries to display in the log window.
   * @return FTXUI element representing the complete UI layout.
   */
  static ftxui::Element build_main_element(
    const std::vector<std::vector<std::string>> & visible_stations_rows,
    const std::vector<std::vector<std::string>> & samples_rows,
    const std::vector<std::vector<std::string>> & solution_rows, ftxui::Element button_column,
    const std::optional<Sophus::SE3d> & deck_pose,
    const std::optional<lighthouse_geometry_utils::AutoCovDiagonal> & deck_auto_cov,
    bool sampling_active, const std::string & message,
    const std::vector<std::string> & log_entries);

  std::function<void()> on_sample_{[] {
    }};                                            ///< Sample button callback.
  std::function<void()> on_solve_{[] {
    }};                                            ///< Solve button callback.
  std::function<void()> on_solve_keypoint_{[] {
    }};                                            ///< Solve (origin @keypoint) button callback.
  std::function<void()> on_save_{[] {
    }};                                            ///< Save button callback.
  std::function<void()> on_clear_samples_{[] {
    }};                                            ///< Clear Samples callback.
  std::function<void()> on_set_keypoint_{[] {
    }};                                            ///< Set keypoint button callback.
  std::function<void()> on_clear_origin_keypoints_{[] {
    }};                                            ///< Clear origin keypoints button callback.
  std::function<void()> on_quit_{[] {
    }};                                            ///< Quit button callback.

  /// Protects table data against concurrent access between
  /// the ROS callbacks (writers) and the renderer lambda (reader).
  mutable std::mutex data_mutex_;

  std::vector<std::vector<std::string>>
  visible_stations_rows_;                                ///< Rows for the Visible Stations table.
  std::vector<std::vector<std::string>> samples_rows_;   ///< Rows for the Samples table.
  std::vector<std::vector<std::string>> solution_rows_;  ///< Rows for the Solution table.
  std::vector<std::string> log_entries_{};    ///< Timestamped log lines for the log window.
  bool sampling_active_{false};               ///< Whether sampling conditions are met.
  std::string message_{};                     ///< Status message displayed in the UI.
  std::string focused_button_description_{};  ///< Description of the currently focused button.
  std::optional<Sophus::SE3d> deck_pose_;                ///< Current deck pose estimate.
  std::optional<lighthouse_geometry_utils::AutoCovDiagonal>
  deck_auto_cov_;                                        ///< Deck pose autocovariance diagonal.

  ftxui::ScreenInteractive screen_;     ///< FTXUI interactive screen instance.
  ftxui::Component btn_sample_;         ///< Sample button component.
  ftxui::Component btn_solve_;          ///< Solve button component.
  ftxui::Component btn_solve_keypoint_;  ///< Solve (origin @keypoint) button component.
  ftxui::Component btn_save_;           ///< Save button component.
  ftxui::Component btn_clear_samples_;  ///< Clear Samples button component.
  ftxui::Component btn_set_keypoint_;   ///< Set keypoint button component.
  ftxui::Component btn_clear_origin_keypoints_;  ///< Clear origin keypoints button component.
  ftxui::Component btn_quit_;           ///< Quit button component.
  ftxui::Component button_bar_;         ///< Horizontal container for buttons.
  ftxui::Component layout_;             ///< Top-level layout component.

  std::thread screen_thread_;  ///< Background thread running the event loop.
};

}  // namespace lighthouse_station_mapper

#endif  // LIGHTHOUSE_STATION_MAPPER__MAPPER_SCREEN_RENDERER_HPP_
