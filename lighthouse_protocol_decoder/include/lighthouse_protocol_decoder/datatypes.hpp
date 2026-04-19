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

#ifndef LIGHTHOUSE_PROTOCOL_DECODER__DATATYPES_HPP_
#define LIGHTHOUSE_PROTOCOL_DECODER__DATATYPES_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "lighthouse_protocol_decoder/constants.hpp"

namespace lighthouse_protocol_decoder
{

class LoggerInterface;   // Forward declaration
class DataFrameContents;  // Forward declaration

/// Callback function invoked when a sync frame is detected
using SyncFrameDetectedCallback = std::function<void ()>;

/// Callback function invoked when a data frame is decoded
/// @param good_sync Indicates if synchronization is good
/// @param frame_data The decoded frame contents
using DataFrameCallback = std::function<void (bool, const DataFrameContents &)>;

/// Raw measurement from a single sensor
struct SensorRawMeasurement
{
  /// Normalized offset value in ticks
  std::uint32_t normalized_offset{0};
};

/// Complete sweep data from all sensors for one sweep
struct SweepBlockRawData
{
  /// Base station ID
  std::uint8_t base_station_id{0};

  /// Timestamp of the sweep
  std::uint32_t timestamp{0};

  /// Raw measurements from all sensors
  std::array<SensorRawMeasurement, kPulseProcessorNSensors> sensors;
};

/// Bearing angles for a single sensor
struct SingleSensorBearing
{
  /// Azimuth angle in degrees
  double azimuth{0.0};

  /// Elevation angle in degrees
  double elevation{0.0};
};

/// Bearing measurements for all sensors from a matched sweep pair
struct SweepBlockBearings
{
  /// Base station ID
  std::uint8_t base_station_id{0};

  /// Hardware timestamp
  std::uint32_t hardware_timestamp{0};

  /// Bearing angles for all sensors
  std::array<SingleSensorBearing, kPulseProcessorNSensors> sensor_angles;
};

/// Callback function invoked when a complete sweep is decoded
using SweepCallback = std::function<void (const SweepBlockRawData &)>;

/// Callback function invoked when bearing measurements are available
using BearingCallback = std::function<void (const SweepBlockBearings &)>;

/// Contents of a decoded data frame from the Lighthouse protocol
struct DataFrameContents
{
  /// Sensor ID (0-3)
  std::uint8_t sid{0};

  /// Polynomial value containing channel, validity, and slow bit
  std::uint8_t npoly{0};

  /// Width of the sweep in ticks
  std::uint16_t width{0};

  /// Offset from sync to sweep in ticks (24 MHz clock)
  std::uint32_t sync_offset{0};

  /// Padding field 1 (should be 0)
  std::uint8_t padding_1{0};

  /// Beam word containing sweep information
  std::uint32_t beam_word{0};

  /// Padding field 2 (should be 0)
  std::uint8_t padding_2{0};

  /// Timestamp counter value (24-bit)
  std::uint32_t timestamp{0};

  /// Raw frame data (12 bytes)
  std::vector<std::uint8_t> raw_data;

  /// Check if the validity bit for the npoly value is set
  /// @return true if npoly is valid
  bool validNpoly() const;

  /// Get the slow bit from the npoly value
  /// @return The slow bit (0 or 1)
  std::uint8_t slowBit() const;

  /// Get the base station ID from the npoly value
  /// @return The base station ID
  std::uint8_t baseStationId() const;
};

}    // namespace lighthouse_protocol_decoder

#endif  // LIGHTHOUSE_PROTOCOL_DECODER__DATATYPES_HPP_
