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

#include "lighthouse_protocol_decoder/measurement_processor.hpp"

#include <cmath>

namespace lighthouse_protocol_decoder {

MeasurementProcessor::MeasurementProcessor(BearingCallback callback,
                                           LoggerInterface::Ptr logger)
    : callback_(std::move(callback)), logger_(std::move(logger)) {}

void MeasurementProcessor::processBlock(
    const SweepBlockRawData &sweep_contents) {
  const auto base_station_id = sweep_contents.base_station_id;

  // Initialize buffer for this base station if needed
  if (per_channel_buffer_.find(base_station_id) == per_channel_buffer_.end()) {
    per_channel_buffer_[base_station_id] = std::deque<SweepBlockRawData>();
  }

  // Add the new block
  per_channel_buffer_[base_station_id].push_back(sweep_contents);

  // Keep only the last 2 blocks
  if (per_channel_buffer_[base_station_id].size() > 2) {
    per_channel_buffer_[base_station_id].pop_front();
  }

  // Need at least 2 blocks to form a pair
  if (per_channel_buffer_[base_station_id].size() < 2) {
    return;
  }

  const auto &current = per_channel_buffer_[base_station_id].back();
  const auto &previous = per_channel_buffer_[base_station_id].front();

  // Check if they form a valid matched pair
  if (!blocksAreMatchedPair(current, previous)) {
    return;
  }

  // Extract and report measurements
  const auto sensor_bearings =
      extractMeasurements(current, previous, base_station_id);

  if (callback_) {
    callback_(sensor_bearings);
  }
}

void MeasurementProcessor::reset() { per_channel_buffer_.clear(); }

bool MeasurementProcessor::blocksAreMatchedPair(
    const SweepBlockRawData &current, const SweepBlockRawData &previous) const {
  // The current block's offset must be greater than the previous
  if (previous.sensors[0].normalized_offset >
      current.sensors[0].normalized_offset) {
    return false;
  }

  // The time difference between blocks should be less than ~180 degrees
  const auto block_delta_timestamp =
      timestampDiff(current.timestamp, previous.timestamp);

  if (block_delta_timestamp > kMaxTimestampDiffForBlockMatch) {
    return false;
  }

  return true;
}

SweepBlockBearings
MeasurementProcessor::extractMeasurements(const SweepBlockRawData &current,
                                          const SweepBlockRawData &previous,
                                          std::uint8_t base_station_id) const {
  SweepBlockBearings sensor_bearings;
  sensor_bearings.base_station_id = base_station_id;
  sensor_bearings.hardware_timestamp = current.timestamp;

  // Get the period for this base station
  const auto channel_period = kBasestationPeriods.at(base_station_id);

  // Calculate bearings for each sensor
  for (std::size_t i = 0; i < kPulseProcessorNSensors; ++i) {
    const auto offset_0 = previous.sensors[i].normalized_offset;
    const auto offset_1 = current.sensors[i].normalized_offset;

    // Convert offsets to phase angles
    const auto phase_beam_0 = (offset_0 / channel_period) * 2.0 * M_PI;
    const auto phase_beam_1 = (offset_1 / channel_period) * 2.0 * M_PI;

    // Calculate polar bearings
    const auto [azimuth_rad, elevation_rad] =
        calculatePolarBearing(phase_beam_0, phase_beam_1);

    // Convert to degrees
    const auto azimuth_deg = azimuth_rad * 180.0 / M_PI;
    const auto elevation_deg = elevation_rad * 180.0 / M_PI;

    sensor_bearings.sensor_angles[i].azimuth = azimuth_deg;
    sensor_bearings.sensor_angles[i].elevation = elevation_deg;
  }

  return sensor_bearings;
}

std::pair<double, double>
MeasurementProcessor::calculatePolarBearing(double phase_beam_0,
                                            double phase_beam_1) const {
  // Calculate azimuth
  const auto azimuth = ((phase_beam_0 + phase_beam_1) / 2.0) - M_PI;

  // Calculate elevation
  const auto p = M_PI / 3.0; // 60 degrees in radians
  const auto beta =
      (phase_beam_1 - phase_beam_0) - (2.0 * M_PI / 3.0); // 120 degrees

  const auto elevation = std::atan(std::sin(beta / 2.0) / std::tan(p / 2.0));

  return {azimuth, elevation};
}

} // namespace lighthouse_protocol_decoder
