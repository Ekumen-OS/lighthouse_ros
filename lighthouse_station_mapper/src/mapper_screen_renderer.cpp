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

#include "lighthouse_station_mapper/mapper_screen_renderer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <functional>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/dom/table.hpp>
#include <sophus/se3.hpp>

namespace lighthouse_station_mapper
{

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;

std::string fmt_double(double v, int precision = 4)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << v;
  return oss.str();
}

/// Format a log entry with a [HH:MM:SS] timestamp prefix.
std::string make_log_entry(const std::string & message)
{
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  localtime_r(&t, &tm_buf);
  std::ostringstream oss;
  oss << '[' << std::put_time(&tm_buf, "%H:%M:%S") << "] " << message;
  return oss.str();
}

/// Create a button with an optional shortcut letter rendered in bold+underline.
/// If @p shortcut_pos is std::string::npos no letter is highlighted.
ftxui::Component make_button(
  const std::string & label, std::function<void()> callback,
  std::size_t shortcut_pos = std::string::npos,
  std::string description = {},
  std::string * help_ptr = nullptr)
{
  auto option = ftxui::ButtonOption::Simple();
  option.transform = [label, shortcut_pos, description, help_ptr](const ftxui::EntryState & s) {
      if (s.focused && help_ptr) {
        *help_ptr = description;
      }
      ftxui::Element content;
      if (shortcut_pos < label.size()) {
        content = ftxui::hbox(
          {
            ftxui::text(label.substr(0, shortcut_pos)),
            ftxui::text(label.substr(shortcut_pos, 1)) | ftxui::bold | ftxui::underlined,
            ftxui::text(label.substr(shortcut_pos + 1)),
          });
      } else {
        content = ftxui::text(label);
      }
      auto element = content | ftxui::border;
      if (s.focused) {
        element |= ftxui::inverted;
      }
      return element;
    };
  return ftxui::Button(label, std::move(callback), std::move(option));
}

}  // namespace

ftxui::Element MapperScreenRenderer::render_table(
  const std::vector<std::vector<std::string>> & rows, const std::string & title)
{
  auto table = ftxui::Table(rows);
  table.SelectAll().Border(ftxui::LIGHT);
  table.SelectRow(0).Decorate(ftxui::bold);
  table.SelectRow(0).SeparatorVertical(ftxui::LIGHT);
  table.SelectRow(0).Border(ftxui::LIGHT);

  // Make all columns uniform width
  if (!rows.empty() && !rows[0].empty()) {
    const int num_columns = static_cast<int>(rows[0].size());
    for (int i = 0; i < num_columns; ++i) {
      table.SelectColumn(i).Decorate(ftxui::flex_grow);
    }
  }

  return ftxui::vbox(
    {
      ftxui::text(title) | ftxui::bold,
      table.Render() | ftxui::xflex | ftxui::vscroll_indicator | ftxui::frame,
    }) |
         ftxui::flex;
}

ftxui::Element MapperScreenRenderer::build_main_element(
  const std::vector<std::vector<std::string>> & visible_stations_rows,
  const std::vector<std::vector<std::string>> & samples_rows,
  const std::vector<std::vector<std::string>> & solution_rows, ftxui::Element button_column,
  const std::optional<Sophus::SE3d> & deck_pose,
  const std::optional<lighthouse_geometry_utils::AutoCovDiagonal> & deck_auto_cov,
  bool sampling_active, const std::string & message,
  const std::vector<std::string> & log_entries)
{
  auto status_indicator = ftxui::hbox(
    {ftxui::text("Sampling: "),
      ftxui::text(sampling_active ? "READY TO SAMPLE" : "NOT READY") |
      (sampling_active ? ftxui::color(ftxui::Color::Green) : ftxui::color(ftxui::Color::Red))});
  auto message_line = ftxui::text(message.empty() ? " " : message);

  // Build the deck pose table
  std::vector<std::vector<std::string>> deck_pose_rows;
  deck_pose_rows.push_back(
    {"X (m)", "Y (m)", "Z (m)", "Roll (°)", "Pitch (°)", "Yaw (°)"});
  if (deck_pose.has_value() && deck_auto_cov.has_value()) {
    const auto & p = deck_pose.value();
    const auto t = p.translation();
    // Sophus has no direct RPY extraction; Eigen's eulerAngles() constrains
    // the middle angle to [0, pi] which gives unexpected pitch values.
    // Manual atan2 gives standard aerospace ZYX convention with correct ranges.
    const auto rm = p.rotationMatrix();
    const double roll = std::atan2(rm(2, 1), rm(2, 2));
    const double pitch = std::atan2(
      -rm(2, 0),
      std::sqrt(rm(2, 1) * rm(2, 1) + rm(2, 2) * rm(2, 2)));
    const double yaw = std::atan2(rm(1, 0), rm(0, 0));

    deck_pose_rows.push_back(
      {
        fmt_double(t.x()),
        fmt_double(t.y()),
        fmt_double(t.z()),
        fmt_double(roll * kRadToDeg),
        fmt_double(pitch * kRadToDeg),
        fmt_double(yaw * kRadToDeg),
      });
  }

  return ftxui::vbox(
    {
      ftxui::text("Lighthouse Station Mapper") | ftxui::bold | ftxui::center,
      ftxui::hbox({status_indicator, ftxui::text("  "), message_line}),
      ftxui::separator(),
      ftxui::hbox(
      {
        button_column,
        ftxui::separator(),
        ftxui::vbox(
        {
          render_table(visible_stations_rows, "Visible Stations"),
          ftxui::separator(),
          render_table(samples_rows, "Samples"),
          ftxui::separator(),
          render_table(solution_rows, "Solution"),
          ftxui::separator(),
          render_table(deck_pose_rows, "Deck Pose"),
          ftxui::separator(),
          [&log_entries]() {
            std::vector<ftxui::Element> lines;
            lines.reserve(log_entries.size());
            for (const auto & entry : log_entries) {
              lines.push_back(ftxui::text(entry));
            }
            if (lines.empty()) {
              lines.push_back(ftxui::text("No log entries yet.") | ftxui::dim);
            } else {
              lines.back() = lines.back() | ftxui::focus;
            }
            return ftxui::vbox(
            {
              ftxui::text("Log") | ftxui::bold,
              ftxui::vbox(std::move(lines)) |
              ftxui::vscroll_indicator | ftxui::frame | ftxui::xflex |
              ftxui::size(ftxui::HEIGHT, ftxui::EQUAL, 8),
            });
          }(),
        }) |
        ftxui::flex,
      }) |
      ftxui::flex,
    }) |
         ftxui::border;
}

MapperScreenRenderer::MapperScreenRenderer()
: visible_stations_rows_{{"Station ID", "Az 0", "El 0", "Az 1", "El 1",
      "Az 2", "El 2", "Az 3", "El 3", "Count",
      "Spread"}},
  samples_rows_{{"#", "Station ID", "Az 0", "El 0", "Az 1", "El 1", "Az 2",
      "El 2", "Az 3", "El 3"}},
  solution_rows_{{"Station ID", "X (m)", "Y (m)", "Z (m)", "Roll (°)",
      "Pitch (°)", "Yaw (°)"}},
  screen_(ftxui::ScreenInteractive::Fullscreen())
{
  btn_sample_ = make_button(
    "Sample",
    [this] {on_sample_();}, 0,
    "Capture a measurement snapshot from all currently visible stations. "
    "The status bar must show READY TO SAMPLE before recording.",
    &focused_button_description_);
  btn_solve_ = make_button(
    "Solve (origin @station)",
    [this] {on_solve_();}, 3,
    "Compute station positions and orientations, placing station 0 at the coordinate origin. "
    "Requires at least two samples from different deck positions.",
    &focused_button_description_);
  btn_solve_keypoint_ = make_button(
    "Solve (origin @keypoint)",
    [this] {on_solve_keypoint_();}, 18,
    "Compute station positions and orientations using 3 keypoints to define the world frame. "
    "The first keypoint sets the origin, the second defines the +X direction, and the third "
    "defines the XY plane. Requires at least two samples and all three keypoints.",
    &focused_button_description_);
  btn_save_ = make_button(
    "Save map", [this] {on_save_();}, std::string::npos,
    "Write the current station geometry solution to disk as a map file.",
    &focused_button_description_);
  btn_set_keypoint_ = make_button(
    "Set origin keypoint",
    [this] {on_set_keypoint_();}, 4,
    "Record the current deck pose as a keypoint for origin definition. "
    "Three presses are needed: the 1st sets the origin, the 2nd points along +X, and the 3rd "
    "defines the XY plane. Requires a prior solve (origin at station) so deck positions are "
    "known. Use Clear origin keypoints to start over.",
    &focused_button_description_);
  btn_clear_samples_ = make_button(
    "Clear samples", [this] {on_clear_samples_();}, 0,
    "Delete all recorded measurement samples and discard the current solution.",
    &focused_button_description_);
  btn_clear_origin_keypoints_ = make_button(
    "Clear origin keypoints", [this] {on_clear_origin_keypoints_();}, 6,
    "Remove all recorded origin keypoints and reset the keypoint-based frame definition.",
    &focused_button_description_);
  btn_quit_ = make_button(
    "Quit", [this] {on_quit_();}, 0,
    "Close the station mapper and exit.",
    &focused_button_description_);

  button_bar_ = ftxui::Container::Vertical(
    {
      btn_sample_,
      btn_solve_,
      btn_solve_keypoint_,
      btn_save_,
      btn_set_keypoint_,
      btn_clear_samples_,
      btn_clear_origin_keypoints_,
      btn_quit_,
    });

  layout_ = ftxui::Renderer(
    button_bar_, [this] {
      std::lock_guard<std::mutex> lock(data_mutex_);
      focused_button_description_.clear();
      auto button_panel = ftxui::vbox(
      {
        btn_sample_->Render(),
        btn_solve_->Render(),
        btn_solve_keypoint_->Render(),
        btn_save_->Render(),
        btn_set_keypoint_->Render(),
        btn_clear_samples_->Render(),
        btn_clear_origin_keypoints_->Render(),
        btn_quit_->Render(),
      });
      auto help_panel = ftxui::vbox(
      {
        ftxui::text(" Help") | ftxui::bold,
        ftxui::separator(),
        ftxui::paragraph(
          focused_button_description_.empty() ?
          "Focus a button to see its description." :
          focused_button_description_),
      }) | ftxui::flex;
      auto button_column = ftxui::vbox({button_panel, ftxui::separator(), help_panel}) |
      ftxui::size(ftxui::WIDTH, ftxui::EQUAL, 30);
      return build_main_element(
        visible_stations_rows_, samples_rows_, solution_rows_, button_column,
        deck_pose_, deck_auto_cov_, sampling_active_, message_, log_entries_);
    });

  layout_ = ftxui::CatchEvent(
    layout_, [this](ftxui::Event event) {
      if (event == ftxui::Event::Character('s') || event == ftxui::Event::Character('S')) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        on_sample_();
        return true;
      }
      if (event == ftxui::Event::Character('v') || event == ftxui::Event::Character('V')) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        on_solve_();
        return true;
      }
      if (event == ftxui::Event::Character('p') || event == ftxui::Event::Character('P')) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        on_solve_keypoint_();
        return true;
      }
      if (event == ftxui::Event::Character('k') || event == ftxui::Event::Character('K')) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        on_set_keypoint_();
        return true;
      }
      if (event == ftxui::Event::Character('o') || event == ftxui::Event::Character('O')) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        on_clear_origin_keypoints_();
        return true;
      }
      if (event == ftxui::Event::Character('c') || event == ftxui::Event::Character('C')) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        on_clear_samples_();
        return true;
      }
      if (event == ftxui::Event::Character('q') || event == ftxui::Event::Character('Q')) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        on_quit_();
        return true;
      }
      return false;
    });
}

MapperScreenRenderer::~MapperScreenRenderer()
{
  stop();
}

void MapperScreenRenderer::start()
{
  screen_thread_ = std::thread(
    [this] {
      screen_.Loop(layout_);
    });
}

void MapperScreenRenderer::stop()
{
  screen_.ExitLoopClosure()();
  if (screen_thread_.joinable()) {
    screen_thread_.join();
  }
}

void MapperScreenRenderer::set_visible_stations(const std::vector<SummarizedStationData> & stations)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Sort by station_id
  auto sorted_stations = stations;
  std::sort(
    sorted_stations.begin(), sorted_stations.end(),
    [](const SummarizedStationData & a, const SummarizedStationData & b) {
      return a.station_id < b.station_id;
    });

  std::vector<std::vector<std::string>> rows;
  rows.push_back(
    {"Station ID", "Az 0 (°)", "El 0 (°)", "Az 1 (°)", "El 1 (°)", "Az 2 (°)", "El 2 (°)",
      "Az 3 (°)", "El 3 (°)", "Count",
      "Spread (°)"});
  for (const auto & s : sorted_stations) {
    rows.push_back(
      {
        std::to_string(s.station_id),
        fmt_double(s.azimuth[0] * kRadToDeg),
        fmt_double(s.elevation[0] * kRadToDeg),
        fmt_double(s.azimuth[1] * kRadToDeg),
        fmt_double(s.elevation[1] * kRadToDeg),
        fmt_double(s.azimuth[2] * kRadToDeg),
        fmt_double(s.elevation[2] * kRadToDeg),
        fmt_double(s.azimuth[3] * kRadToDeg),
        fmt_double(s.elevation[3] * kRadToDeg),
        std::to_string(s.count),
        fmt_double(s.spread * kRadToDeg, 6),
      });
  }
  visible_stations_rows_ = std::move(rows);
  screen_.PostEvent(ftxui::Event::Custom);
}

void MapperScreenRenderer::set_current_samples(const std::vector<LighthouseSample> & samples)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Sort by deck_pose_id first, then by station_id
  auto sorted_samples = samples;
  std::sort(
    sorted_samples.begin(), sorted_samples.end(),
    [](const LighthouseSample & a, const LighthouseSample & b) {
      if (a.deck_pose_id != b.deck_pose_id) {
        return a.deck_pose_id < b.deck_pose_id;
      }
      return a.station_id < b.station_id;
    });

  std::vector<std::vector<std::string>> rows;
  rows.push_back(
    {"Sample", "Station ID", "Az 0 (°)", "El 0 (°)", "Az 1 (°)", "El 1 (°)", "Az 2 (°)", "El 2 (°)",
      "Az 3 (°)", "El 3 (°)"});
  for (std::size_t i = 0; i < sorted_samples.size(); ++i) {
    const auto & s = sorted_samples[i];
    rows.push_back(
      {
        std::to_string(s.deck_pose_id),
        std::to_string(s.station_id),
        fmt_double(s.azimuth[0] * kRadToDeg),
        fmt_double(s.elevation[0] * kRadToDeg),
        fmt_double(s.azimuth[1] * kRadToDeg),
        fmt_double(s.elevation[1] * kRadToDeg),
        fmt_double(s.azimuth[2] * kRadToDeg),
        fmt_double(s.elevation[2] * kRadToDeg),
        fmt_double(s.azimuth[3] * kRadToDeg),
        fmt_double(s.elevation[3] * kRadToDeg),
      });
  }
  samples_rows_ = std::move(rows);
  screen_.PostEvent(ftxui::Event::Custom);
}

void MapperScreenRenderer::set_station_poses(
  const lighthouse_geometry_utils::StationPoseEstimates & result)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // Create pairs of (station_id, index) for sorting
  std::vector<std::pair<lighthouse_geometry_utils::StationId, std::size_t>> indexed_stations;
  indexed_stations.reserve(result.station_ids.size());
  for (std::size_t i = 0; i < result.station_ids.size(); ++i) {
    indexed_stations.emplace_back(result.station_ids[i], i);
  }

  // Sort by station_id
  std::sort(
    indexed_stations.begin(), indexed_stations.end(),
    [](const auto & a, const auto & b) {
      return a.first < b.first;
    });

  std::vector<std::vector<std::string>> rows;
  rows.push_back({"Station ID", "X (m)", "Y (m)", "Z (m)", "Roll (°)", "Pitch (°)", "Yaw (°)"});

  for (const auto & [station_id, idx] : indexed_stations) {
    const auto & pose = result.station_poses[idx];

    const auto translation = pose.translation();
    const auto rotation_matrix = pose.rotationMatrix();

    // Extract Euler angles (ZYX convention: yaw-pitch-roll)
    const double roll = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
    const double pitch = std::atan2(
      -rotation_matrix(2, 0), std::sqrt(
        rotation_matrix(2, 1) * rotation_matrix(2, 1) +
        rotation_matrix(2, 2) * rotation_matrix(2, 2)));
    const double yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

    // Convert radians to degrees for angles
    const double roll_deg = roll * kRadToDeg;
    const double pitch_deg = pitch * kRadToDeg;
    const double yaw_deg = yaw * kRadToDeg;

    rows.push_back(
      {
        std::to_string(station_id),
        fmt_double(translation.x()),
        fmt_double(translation.y()),
        fmt_double(translation.z()),
        fmt_double(roll_deg),
        fmt_double(pitch_deg),
        fmt_double(yaw_deg),
      });
  }
  solution_rows_ = std::move(rows);
  screen_.PostEvent(ftxui::Event::Custom);
}

void MapperScreenRenderer::set_sample_callback(std::function<void()> cb)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  on_sample_ = std::move(cb);
}

void MapperScreenRenderer::set_solve_callback(std::function<void()> cb)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  on_solve_ = std::move(cb);
}

void MapperScreenRenderer::set_solve_keypoint_callback(std::function<void()> cb)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  on_solve_keypoint_ = std::move(cb);
}

void MapperScreenRenderer::set_save_callback(std::function<void()> cb)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  on_save_ = std::move(cb);
}

void MapperScreenRenderer::set_clear_samples_callback(std::function<void()> cb)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  on_clear_samples_ = std::move(cb);
}

void MapperScreenRenderer::set_set_keypoint_callback(std::function<void()> cb)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  on_set_keypoint_ = std::move(cb);
}

void MapperScreenRenderer::set_clear_origin_keypoints_callback(std::function<void()> cb)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  on_clear_origin_keypoints_ = std::move(cb);
}

void MapperScreenRenderer::set_quit_callback(std::function<void()> cb)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  on_quit_ = std::move(cb);
}

void MapperScreenRenderer::set_sampling_status(bool active)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  sampling_active_ = active;
  screen_.PostEvent(ftxui::Event::Custom);
}

void MapperScreenRenderer::set_message(std::string message)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  append_log_locked(make_log_entry(message));
  message_ = std::move(message);
  screen_.PostEvent(ftxui::Event::Custom);
}

void MapperScreenRenderer::set_deck_pose(
  const Sophus::SE3d & pose,
  const lighthouse_geometry_utils::AutoCovDiagonal & auto_cov)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  deck_pose_ = pose;
  deck_auto_cov_ = auto_cov;
  screen_.PostEvent(ftxui::Event::Custom);
}

void MapperScreenRenderer::clear_deck_pose()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  deck_pose_.reset();
  deck_auto_cov_.reset();
  screen_.PostEvent(ftxui::Event::Custom);
}

void MapperScreenRenderer::append_log(std::string message)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  append_log_locked(make_log_entry(std::move(message)));
  screen_.PostEvent(ftxui::Event::Custom);
}

void MapperScreenRenderer::append_log_locked(std::string entry)
{
  constexpr std::size_t kMaxLogEntries = 1000;
  if (log_entries_.size() >= kMaxLogEntries) {
    log_entries_.erase(log_entries_.begin());
  }
  log_entries_.push_back(std::move(entry));
}

}  // namespace lighthouse_station_mapper
