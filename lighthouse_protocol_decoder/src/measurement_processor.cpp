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

#include <algorithm>
#include <cmath>

#include "lighthouse_geometry_utils/station_angles_utils.hpp"

namespace lighthouse_protocol_decoder
{

MeasurementProcessor::MeasurementProcessor(
  BearingCallback callback,
  LoggerInterface::Ptr logger)
: callback_(std::move(callback)),
  logger_(logger ? std::move(logger) : std::make_shared<NullLogger>()) {}

void MeasurementProcessor::processBlock(
  const SweepBlockRawData & sweep_contents)
{
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

  const auto & current = per_channel_buffer_[base_station_id].back();
  const auto & previous = per_channel_buffer_[base_station_id].front();

  // Check if they form a valid matched pair
  if (!blocksAreMatchedPair(current, previous)) {
    return;
  }

  // Extract and report measurements
  const auto sensor_bearings =
    extractMeasurements(current, previous, base_station_id);

  // Clear the buffer after a successful match so the next block starts a fresh
  // pair
  per_channel_buffer_[base_station_id].clear();

  if (!bearingsAreValid(sensor_bearings)) {
    return;
  }

  if (callback_) {
    callback_(sensor_bearings);
  }
}

void MeasurementProcessor::reset() {per_channel_buffer_.clear();}

bool MeasurementProcessor::blocksAreMatchedPair(
  const SweepBlockRawData & current, const SweepBlockRawData & previous) const
{
  // Check if they're from the same rotation using timestamp0 (rotor zero crossing time)
  // This matches the Crazyflie firmware approach with a tight tolerance of 100 ticks (~4.2 μs)
  const auto timestamp0_diff = timestampDiff(current.timestamp0, previous.timestamp0);

  if (timestamp0_diff > 100) {
    logger_->debug(
      "Sweep pair rejected: timestamp0 mismatch (" +
      std::to_string(timestamp0_diff) +
      " ticks) - sweeps not from same rotation");
    return false;
  }

  return true;
}

SweepBlockBearings
MeasurementProcessor::extractMeasurements(
  const SweepBlockRawData & current,
  const SweepBlockRawData & previous,
  std::uint8_t base_station_id) const
{
  SweepBlockBearings sensor_bearings;
  sensor_bearings.base_station_id = base_station_id;
  sensor_bearings.hardware_timestamp = current.timestamp;

  // Get the period for this base station
  const auto channel_period = kBasestationPeriods.at(base_station_id);

  // Calculate bearings for each sensor
  for (std::size_t i = 0; i < kPulseProcessorNSensors; ++i) {
    const auto offset_0 = previous.sensors[i].normalized_offset;
    const auto offset_1 = current.sensors[i].normalized_offset;

    // Step 1: Convert offsets to phase angles
    const auto [phase_0, phase_1] = calculatePhaseAngles(offset_0, offset_1, channel_period);

    // Steps 2-4: Convert phase angles through V2 → V1 → spherical coordinates
    const auto [azimuth_rad, elevation_rad] = calculatePolarBearing(phase_0, phase_1);

    // Convert to degrees for output
    const auto azimuth_deg = azimuth_rad * 180.0 / M_PI;
    const auto elevation_deg = elevation_rad * 180.0 / M_PI;

    sensor_bearings.sensor_angles[i].azimuth = azimuth_deg;
    sensor_bearings.sensor_angles[i].elevation = elevation_deg;
  }

  return sensor_bearings;
}

bool MeasurementProcessor::bearingsAreValid(const SweepBlockBearings & bearings) const
{
  // Physical constraint: at a reference distance, the angular spread across
  // sensors must be consistent with the sensor's physical baseline.
  // The maximum angle range is atan(kMaxSensorBaseline / kReferenceDistance).
  constexpr double kReferenceDistance = 1.5;     // meters
  constexpr double kMaxSensorBaseline = 0.0335;  // meters
  const double max_angle_range_deg =
    std::atan(kMaxSensorBaseline / kReferenceDistance) * 180.0 / M_PI;

  const auto az_mm = std::minmax_element(
    bearings.sensor_angles.begin(), bearings.sensor_angles.end(),
    [](const auto & a, const auto & b) {return a.azimuth < b.azimuth;});
  const auto el_mm = std::minmax_element(
    bearings.sensor_angles.begin(), bearings.sensor_angles.end(),
    [](const auto & a, const auto & b) {return a.elevation < b.elevation;});

  const double az_range_deg = az_mm.second->azimuth - az_mm.first->azimuth;
  const double el_range_deg = el_mm.second->elevation - el_mm.first->elevation;

  if (az_range_deg > max_angle_range_deg) {
    logger_->warning(
      "Bearing rejected: azimuth spread across sensors (" +
      std::to_string(az_range_deg) +
      " deg) exceeds physical limit (" +
      std::to_string(max_angle_range_deg) + " deg)");
    return false;
  }

  if (el_range_deg > max_angle_range_deg) {
    logger_->warning(
      "Bearing rejected: elevation spread across sensors (" +
      std::to_string(el_range_deg) +
      " deg) exceeds physical limit (" +
      std::to_string(max_angle_range_deg) + " deg)");
    return false;
  }

  // Winding order check: traversing sensors in the physical order 0, 1, 3, 2
  // must form a clockwise polygon in (azimuth, elevation) space when the
  // station faces the deck from above. Computed via the shoelace formula;
  // a negative signed area indicates clockwise winding.
  constexpr std::array<std::size_t, 4> kSensorWindingOrder = {0, 1, 3, 2};
  double signed_area_2 = 0.0;
  for (std::size_t k = 0; k < kSensorWindingOrder.size(); ++k) {
    const auto & curr = bearings.sensor_angles[kSensorWindingOrder[k]];
    const auto & next =
      bearings.sensor_angles[kSensorWindingOrder[(k + 1) % kSensorWindingOrder.size()]];
    signed_area_2 += curr.azimuth * next.elevation - next.azimuth * curr.elevation;
  }

  if (signed_area_2 > 0.0) {
    logger_->warning(
      "Bearing rejected: sensor winding order 0,1,3,2 is not clockwise "
      "(signed area: " + std::to_string(signed_area_2) + " deg^2)");
    return false;
  }

  return true;
}

std::pair<double, double>
MeasurementProcessor::calculatePhaseAngles(
  double offset_0,
  double offset_1,
  double period) const
{
  // Convert normalized timing offsets to phase angles (0 to 2π)
  // representing the rotor angle when each light pulse was detected
  const auto phase_0 = (offset_0 / period) * 2.0 * M_PI;
  const auto phase_1 = (offset_1 / period) * 2.0 * M_PI;

  return {phase_0, phase_1};
}

std::pair<double, double>
MeasurementProcessor::calculatePolarBearing(
  double phase_beam_0,
  double phase_beam_1) const
{
  // Convert phase angles through V2 → V1 → spherical coordinates pipeline
  // Step 1: Convert phase angles to V2 angles (with tilt corrections)
  const auto [v2_angle_1, v2_angle_2] =
    lighthouse_geometry_utils::calculateV2Angles(phase_beam_0, phase_beam_1);

  // Step 2: Convert V2 angles to V1 angles (plane intersection parameterization)
  const auto [angleH, angleV] =
    lighthouse_geometry_utils::calculateV1Angles(v2_angle_1, v2_angle_2);

  // Step 3: Convert V1 angles to true spherical coordinates
  const auto [azimuth, elevation] =
    lighthouse_geometry_utils::convertV1AnglesToSpherical(angleH, angleV);

  return {azimuth, elevation};
}

}    // namespace lighthouse_protocol_decoder
