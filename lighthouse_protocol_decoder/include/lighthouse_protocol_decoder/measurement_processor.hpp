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

#ifndef LIGHTHOUSE_PROTOCOL_DECODER__MEASUREMENT_PROCESSOR_HPP_
#define LIGHTHOUSE_PROTOCOL_DECODER__MEASUREMENT_PROCESSOR_HPP_

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <utility>

#include "lighthouse_protocol_decoder/constants.hpp"
#include "lighthouse_protocol_decoder/datatypes.hpp"
#include "lighthouse_protocol_decoder/logger.hpp"

namespace lighthouse_protocol_decoder
{

/// Processor for converting sweep data into bearing measurements
class MeasurementProcessor
{
public:
  /// Constructor
  /// @param callback Callback invoked when bearing measurements are ready
  /// @param logger Logger instance (optional)
  explicit MeasurementProcessor(
    BearingCallback callback,
    LoggerInterface::Ptr logger = nullptr);

  /// Process a sweep block
  /// @param sweep_contents The sweep block to process
  void processBlock(const SweepBlockRawData & sweep_contents);

  /// Reset the processor state
  void reset();

private:
  /// Check if two consecutive blocks form a matched pair
  /// @param current The current block
  /// @param previous The previous block
  /// @return true if they form a valid pair
  bool blocksAreMatchedPair(
    const SweepBlockRawData & current,
    const SweepBlockRawData & previous) const;

  /// Extract bearing measurements from a matched pair of blocks
  /// @param current The current block (second sweep)
  /// @param previous The previous block (first sweep)
  /// @param base_station_id The base station ID
  /// @return The extracted bearing measurements
  SweepBlockBearings extractMeasurements(
    const SweepBlockRawData & current,
    const SweepBlockRawData & previous,
    std::uint8_t base_station_id) const;

  /// Calculate polar bearing from two phase measurements
  ///
  /// This function converts raw phase measurements to true spherical coordinates
  /// through a multi-step process matching the Crazyflie firmware:
  /// 1. Phase angles → V2 angles (with ±60° tilt corrections)
  /// 2. V2 angles → V1 angles (plane intersection parameterization)
  /// 3. V1 angles → 3D ray direction (cross product of plane normals)
  /// 4. Ray direction → Spherical coordinates (azimuth, elevation)
  ///
  /// @param phase_beam_0 Phase of first beam (radians, 0 to 2π)
  /// @param phase_beam_1 Phase of second beam (radians, 0 to 2π)
  /// @return Pair of (azimuth, elevation) in radians, standard spherical coordinates
  std::pair<double, double> calculatePolarBearing(
    double phase_beam_0,
    double phase_beam_1) const;

  /// Convert timing offsets to phase angles
  /// @param offset_0 First sweep normalized offset (ticks)
  /// @param offset_1 Second sweep normalized offset (ticks)
  /// @param period Rotor period for the base station channel (ticks)
  /// @return Pair of (phase_0, phase_1) in radians [0, 2π)
  std::pair<double, double> calculatePhaseAngles(
    double offset_0,
    double offset_1,
    double period) const;

  /// Convert phase angles to V2 angles with rotor tilt corrections
  /// @param phase_0 First sweep phase angle (radians)
  /// @param phase_1 Second sweep phase angle (radians)
  /// @return Pair of (v2_angle_1, v2_angle_2) in radians [-π, π)
  std::pair<double, double> calculateV2Angles(
    double phase_0,
    double phase_1) const;

  /// Convert V2 angles to V1 angles (plane intersection parameterization)
  /// @param v2_angle_1 First V2 angle (radians)
  /// @param v2_angle_2 Second V2 angle (radians)
  /// @return Pair of (angleH, angleV) - NOT standard spherical coordinates
  std::pair<double, double> calculateV1Angles(
    double v2_angle_1,
    double v2_angle_2) const;

  /// Convert V1 angles to true spherical coordinates
  /// @param angleH Horizontal plane angle (radians)
  /// @param angleV Vertical plane angle (radians)
  /// @return Pair of (azimuth, elevation) in radians, standard spherical coordinates
  std::pair<double, double> convertV1AnglesToSpherical(
    double angleH,
    double angleV) const;

  /// Validate that the spread of bearing angles across sensors is physically
  /// plausible given the sensor's maximum baseline
  /// @param bearings The bearing measurements to validate
  /// @return true if the bearings pass the sanity check
  bool bearingsAreValid(const SweepBlockBearings & bearings) const;

  /// Buffer of sweep blocks per base station (stores last 2 blocks)
  std::map<std::uint8_t, std::deque<SweepBlockRawData>> per_channel_buffer_;

  /// Callback for bearing measurements
  BearingCallback callback_;

  /// Logger instance
  LoggerInterface::Ptr logger_;
};

}    // namespace lighthouse_protocol_decoder

#endif  // LIGHTHOUSE_PROTOCOL_DECODER__MEASUREMENT_PROCESSOR_HPP_
