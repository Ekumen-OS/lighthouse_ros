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

#ifndef LIGHTHOUSE_PROTOCOL_DECODER__SWEEP_PROCESSOR_HPP_
#define LIGHTHOUSE_PROTOCOL_DECODER__SWEEP_PROCESSOR_HPP_

#include <cstdint>
#include <map>
#include <memory>

#include "lighthouse_protocol_decoder/constants.hpp"
#include "lighthouse_protocol_decoder/datatypes.hpp"
#include "lighthouse_protocol_decoder/logger.hpp"

namespace lighthouse_protocol_decoder {

/// Processor for collecting and validating complete sweep data from all sensors
class SweepProcessor {
public:
  /// Constructor
  /// @param callback Callback invoked when a complete sweep is validated
  /// @param logger Logger instance (optional)
  explicit SweepProcessor(SweepCallback callback,
                          LoggerInterface::Ptr logger = nullptr);

  /// Process a data frame
  /// @param frame The data frame to process
  void processFrame(const DataFrameContents &frame);

  /// Reset the processor state
  void reset();

private:
  /// Validate if the current set of frames forms a complete valid sweep
  /// @return true if the sweep is valid
  bool validateSweep() const;

  /// Complete the block information and return the sweep data
  /// @return The complete sweep block data
  SweepBlockRawData completeBlockInformation() const;

  /// Buffer of frames indexed by sensor ID
  std::map<std::uint8_t, DataFrameContents> block_frames_;

  /// Callback for complete sweep data
  SweepCallback callback_;

  /// Logger instance
  LoggerInterface::Ptr logger_;
};

} // namespace lighthouse_protocol_decoder

#endif // LIGHTHOUSE_PROTOCOL_DECODER__SWEEP_PROCESSOR_HPP_
