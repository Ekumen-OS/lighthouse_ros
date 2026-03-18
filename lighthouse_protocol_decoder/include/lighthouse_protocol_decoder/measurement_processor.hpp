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

#include "lighthouse_protocol_decoder/constants.hpp"
#include "lighthouse_protocol_decoder/datatypes.hpp"
#include "lighthouse_protocol_decoder/logger.hpp"

namespace lighthouse_protocol_decoder {

/// Processor for converting sweep data into bearing measurements
class MeasurementProcessor {
public:
  /// Constructor
  /// @param callback Callback invoked when bearing measurements are ready
  /// @param logger Logger instance (optional)
  explicit MeasurementProcessor(BearingCallback callback,
                                LoggerInterface::Ptr logger = nullptr);

  /// Process a sweep block
  /// @param sweep_contents The sweep block to process
  void processBlock(const SweepBlockRawData &sweep_contents);

  /// Reset the processor state
  void reset();

private:
  /// Check if two consecutive blocks form a matched pair
  /// @param current The current block
  /// @param previous The previous block
  /// @return true if they form a valid pair
  bool blocksAreMatchedPair(const SweepBlockRawData &current,
                            const SweepBlockRawData &previous) const;

  /// Extract bearing measurements from a matched pair of blocks
  /// @param current The current block (second sweep)
  /// @param previous The previous block (first sweep)
  /// @param base_station_id The base station ID
  /// @return The extracted bearing measurements
  SweepBlockBearings extractMeasurements(const SweepBlockRawData &current,
                                         const SweepBlockRawData &previous,
                                         std::uint8_t base_station_id) const;

  /// Calculate polar bearing from two phase measurements
  /// @param phase_beam_0 Phase of first beam (radians)
  /// @param phase_beam_1 Phase of second beam (radians)
  /// @return Pair of (azimuth, elevation) in radians
  std::pair<double, double> calculatePolarBearing(double phase_beam_0,
                                                  double phase_beam_1) const;

  /// Buffer of sweep blocks per base station (stores last 2 blocks)
  std::map<std::uint8_t, std::deque<SweepBlockRawData>> per_channel_buffer_;

  /// Callback for bearing measurements
  BearingCallback callback_;

  /// Logger instance
  LoggerInterface::Ptr logger_;
};

} // namespace lighthouse_protocol_decoder

#endif // LIGHTHOUSE_PROTOCOL_DECODER__MEASUREMENT_PROCESSOR_HPP_
