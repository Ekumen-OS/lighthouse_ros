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

#ifndef LIGHTHOUSE_STATION_MAPPER__DATATYPES_HPP_
#define LIGHTHOUSE_STATION_MAPPER__DATATYPES_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <sophus/se3.hpp>

namespace lighthouse_station_mapper
{

using StationId = std::size_t;   ///< Unique identifier for a lighthouse base station.
using DeckPoseId = std::size_t;  ///< Unique identifier for a deck sampling pose.

/// Median-summarized lighthouse readings for one station, computed from
/// a sliding window of raw measurements.
struct SummarizedStationData
{
  /// Identifier of the base station that produced these measurements.
  StationId station_id;
  /// Median elevation per sensor (radians).
  std::array<double, 4> elevation;
  /// Median azimuth per sensor (radians).
  std::array<double, 4> azimuth;
  /// Number of raw samples that contributed to this summary.
  std::size_t count;
  /// Max minus min across all contributing raw angles (radians).
  double spread;
  /// Timestamp of the latest sample in this summary (seconds since epoch).
  double latest_timestamp;
};

/// A committed lighthouse sample: median readings for one station at
/// a particular deck pose.
struct LighthouseSample
{
  /// Identifier of the base station that produced these measurements.
  StationId station_id;
  /// Identifier for the deck pose where this sample was taken.
  DeckPoseId deck_pose_id;
  /// Median elevation per sensor (radians).
  std::array<double, 4> elevation;
  /// Median azimuth per sensor (radians).
  std::array<double, 4> azimuth;
};

}  // namespace lighthouse_station_mapper

#endif  // LIGHTHOUSE_STATION_MAPPER__DATATYPES_HPP_
