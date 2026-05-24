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

#include "lighthouse_protocol_decoder/sweep_processor.hpp"

#include <set>
#include <vector>

namespace lighthouse_protocol_decoder
{

SweepProcessor::SweepProcessor(
  SweepCallback callback,
  LoggerInterface::Ptr logger)
: callback_(std::move(callback)),
  logger_(logger ? std::move(logger) : std::make_shared<NullLogger>()) {}

void SweepProcessor::processFrame(const DataFrameContents & frame)
{
  const auto latest_sensor = frame.sid;
  const auto latest_timestamp = frame.timestamp;

  // Store the frame
  block_frames_[latest_sensor] = frame;

  // Remove any frames too far from latest (can't belong to same sweep)
  // Use absolute difference to handle out-of-order arrival
  std::vector<std::uint8_t> sensors_to_remove;
  for (const auto &[sensor, stored_frame] : block_frames_) {
    if (timestampAbsDiffLargerThan(
        latest_timestamp, stored_frame.timestamp,
        kMaxTimestampDiffForSweep))
    {
      sensors_to_remove.push_back(sensor);
    }
  }

  for (const auto sensor : sensors_to_remove) {
    block_frames_.erase(sensor);
  }

  // Check if we have a complete valid sweep
  if (!validateSweep()) {
    return;
  }

  // Invoke callback with complete sweep data
  if (callback_) {
    callback_(completeBlockInformation());
  }

  // Reset buffer for next sweep
  block_frames_.clear();
}

void SweepProcessor::reset() {block_frames_.clear();}

bool SweepProcessor::validateSweep() const
{
  // Must have data from all 4 sensors
  if (block_frames_.size() != kPulseProcessorNSensors) {
    return false;
  }

  // Validation criteria (matching firmware's processWorkspaceBlock):
  // 1. At least 1 sensor must have valid npoly
  // 2. All sensors with valid npoly must have the same base_station_id
  // 3. Exactly 1 sensor must have a valid offset measurement (sync_offset != 0)

  std::size_t valid_npolys = 0;
  std::size_t valid_offsets = 0;
  std::set<std::uint8_t> channels_seen;

  for (const auto &[sensor, frame] : block_frames_) {
    if (frame.validNpoly()) {
      valid_npolys++;
      channels_seen.insert(frame.baseStationId());
    }
    if (frame.sync_offset != 0) {
      valid_offsets++;
    }
  }

  if (valid_npolys < 1) {
    logger_->debug(
      "Sweep discarded: need at least 1 sensor with valid polynomial, got " +
      std::to_string(valid_npolys));
    return false;
  }

  if (valid_offsets != 1) {
    logger_->debug(
      "Sweep discarded: expected exactly 1 sensor with sync offset, got " +
      std::to_string(valid_offsets));
    return false;
  }

  if (channels_seen.size() != 1) {
    logger_->debug(
      "Sweep discarded: expected frames from a single base station, got " +
      std::to_string(channels_seen.size()) + " distinct channels");
    return false;
  }

  return true;
}

SweepBlockRawData SweepProcessor::completeBlockInformation() const
{
  SweepBlockRawData sweep_contents;

  // Find the base_station_id and reference sensor (the one with sync_offset)
  auto reference_sensor_offset = 0u;
  auto reference_sensor_timestamp = 0u;
  auto found_reference = false;

  for (const auto &[sensor, frame] : block_frames_) {
    if (frame.validNpoly()) {
      sweep_contents.base_station_id = frame.baseStationId();
    }
    if (frame.sync_offset != 0) {
      reference_sensor_offset = frame.sync_offset;
      reference_sensor_timestamp = frame.timestamp;
      found_reference = true;

      // Calculate timestamp0 - the rotor zero crossing time
      // This matches the Crazyflie firmware approach
      sweep_contents.timestamp0 = timestampDiff(
        reference_sensor_timestamp, reference_sensor_offset);
    }
  }

  // Complete the information for all sensors
  auto min_timestamp = reference_sensor_timestamp;
  for (const auto &[sensor, frame] : block_frames_) {
    SensorRawMeasurement sensor_measurement;
    sensor_measurement.normalized_offset = frame.sync_offset;

    if (frame.sync_offset == 0 && found_reference) {
      // Calculate offset relative to the reference sensor
      const auto timestamp_delta =
        timestampDiff(frame.timestamp, reference_sensor_timestamp);
      sensor_measurement.normalized_offset =
        timestampSum(reference_sensor_offset, timestamp_delta);
    }

    sweep_contents.sensors[sensor] = sensor_measurement;

    // Track minimum timestamp
    if (frame.timestamp < min_timestamp) {
      min_timestamp = frame.timestamp;
    }
  }

  // Use the minimum timestamp from all sensors (matching Python implementation)
  // Note: This may fail if timestamp overflows within the sweep
  sweep_contents.timestamp = min_timestamp;

  return sweep_contents;
}

}    // namespace lighthouse_protocol_decoder
