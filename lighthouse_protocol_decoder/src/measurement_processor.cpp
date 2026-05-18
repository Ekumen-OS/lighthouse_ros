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
  // The current block's offset must be greater than the previous
  if (previous.sensors[0].normalized_offset >
    current.sensors[0].normalized_offset)
  {
    logger_->debug(
      "Sweep pair rejected: sensor[0] offset did not increase (" +
      std::to_string(previous.sensors[0].normalized_offset) +
      " -> " +
      std::to_string(current.sensors[0].normalized_offset) + ")");
    return false;
  }

  // The time difference between blocks should be less than ~180 degrees
  const auto block_delta_timestamp =
    timestampDiff(current.timestamp, previous.timestamp);

  if (block_delta_timestamp > kMaxTimestampDiffForBlockMatch) {
    logger_->debug(
      "Sweep pair rejected: timestamp delta between blocks (" +
      std::to_string(block_delta_timestamp) +
      " ticks) exceeds half-rotation limit (" +
      std::to_string(kMaxTimestampDiffForBlockMatch) + " ticks)");
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
MeasurementProcessor::calculateV2Angles(
  double phase_0,
  double phase_1) const
{
  // Apply rotor tilt corrections to raw phase angles
  // V2 base stations have light planes tilted ±30° from vertical
  // The corrections are ±π/3 (60°) to account for both the tilt and 120° rotor offset
  // The -π term centers the range to [-π, π)
  const auto v2_angle_1 = phase_0 - M_PI + M_PI / 3.0;  // First sweep: +60° correction
  const auto v2_angle_2 = phase_1 - M_PI - M_PI / 3.0;  // Second sweep: -60° correction

  return {v2_angle_1, v2_angle_2};
}

std::pair<double, double>
MeasurementProcessor::calculateV1Angles(
  double v2_angle_1,
  double v2_angle_2) const
{
  // Convert V2 angles (tilted planes) to V1 angles (plane intersection parameterization)
  // This follows the Crazyflie firmware conversion in pulse_processor_v2.c
  // Note: angleH and angleV are NOT standard spherical azimuth/elevation

  constexpr double kTiltAngle = M_PI / 6.0;  // 30° - physical tilt of light planes
  const auto tant = std::tan(kTiltAngle);

  // angleH: horizontal plane angle (average of V2 angles)
  const auto angleH = (v2_angle_1 + v2_angle_2) / 2.0;

  // angleV: vertical plane angle (from plane intersection geometry)
  const auto angleV = std::atan2(
    std::sin(v2_angle_2 - v2_angle_1),
    tant * (std::cos(v2_angle_1) + std::cos(v2_angle_2))
  );

  return {angleH, angleV};
}

std::pair<double, double>
MeasurementProcessor::convertV1AnglesToSpherical(
  double angleH,
  double angleV) const
{
  // Convert V1 angles (plane intersection) to standard spherical coordinates
  // Following the Crazyflie firmware approach in lighthouse_geometry.c

  const auto sin_h = std::sin(angleH);
  const auto cos_h = std::cos(angleH);
  const auto sin_v = std::sin(angleV);
  const auto cos_v = std::cos(angleV);

  // Define normal vectors to two perpendicular planes
  // plane_a: normal to vertical plane rotated by angleH around Z-axis
  // plane_b: normal to horizontal plane rotated by angleV around Y-axis
  const std::array<double, 3> plane_a = {sin_h, -cos_h, 0.0};
  const std::array<double, 3> plane_b = {-sin_v, 0.0, cos_v};

  // Ray direction is the cross product: plane_b × plane_a
  std::array<double, 3> raw_ray = {
    plane_b[1] * plane_a[2] - plane_b[2] * plane_a[1],  // = cos_v * cos_h
    plane_b[2] * plane_a[0] - plane_b[0] * plane_a[2],  // = cos_v * sin_h
    plane_b[0] * plane_a[1] - plane_b[1] * plane_a[0]   // = sin_v * cos_h
  };

  // Normalize the ray vector
  const auto ray_length = std::sqrt(
    raw_ray[0] * raw_ray[0] +
    raw_ray[1] * raw_ray[1] +
    raw_ray[2] * raw_ray[2]
  );

  raw_ray[0] /= ray_length;
  raw_ray[1] /= ray_length;
  raw_ray[2] /= ray_length;

  // Convert normalized ray to true spherical coordinates
  // Standard spherical coordinate conversion: (x, y, z) → (azimuth, elevation)
  const auto azimuth = std::atan2(raw_ray[1], raw_ray[0]);
  const auto elevation = std::asin(raw_ray[2]);

  return {azimuth, elevation};
}

std::pair<double, double>
MeasurementProcessor::calculatePolarBearing(
  double phase_beam_0,
  double phase_beam_1) const
{
  // This function is now just an orchestrator that calls the individual steps
  // For backwards compatibility, keeping the same signature

  // Step 1: Phase angles are already provided as input (this step done in extractMeasurements)
  // Step 2: Convert phase angles to V2 angles (with tilt corrections)
  const auto [v2_angle_1, v2_angle_2] = calculateV2Angles(phase_beam_0, phase_beam_1);

  // Step 3: Convert V2 angles to V1 angles (plane intersection parameterization)
  const auto [angleH, angleV] = calculateV1Angles(v2_angle_1, v2_angle_2);

  // Step 4: Convert V1 angles to true spherical coordinates
  const auto [azimuth, elevation] = convertV1AnglesToSpherical(angleH, angleV);

  return {azimuth, elevation};
}

}    // namespace lighthouse_protocol_decoder
