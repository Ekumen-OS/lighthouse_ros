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

#ifndef LIGHTHOUSE_PROTOCOL_DECODER__LIGHTHOUSE_PROTOCOL_DECODER_HPP_
#define LIGHTHOUSE_PROTOCOL_DECODER__LIGHTHOUSE_PROTOCOL_DECODER_HPP_

#include <array>
#include <cstdint>
#include <memory>

#include "lighthouse_protocol_decoder/constants.hpp"
#include "lighthouse_protocol_decoder/data_frame_decoder.hpp"
#include "lighthouse_protocol_decoder/datatypes.hpp"
#include "lighthouse_protocol_decoder/logger.hpp"
#include "lighthouse_protocol_decoder/measurement_processor.hpp"
#include "lighthouse_protocol_decoder/ootx_frame_decoder.hpp"
#include "lighthouse_protocol_decoder/sweep_processor.hpp"
#include "lighthouse_protocol_decoder/sync_frame_decoder.hpp"

namespace lighthouse_protocol_decoder {

/// Main decoder for the Lighthouse protocol
class LighthouseProtocolDecoder {
public:
  /// Operating modes
  enum class Mode { SYNC, DATA };

  /// Constructor
  /// @param app_bearing_callback Callback for bearing measurements (optional)
  /// @param logger Logger instance (optional)
  explicit LighthouseProtocolDecoder(
      BearingCallback app_bearing_callback = nullptr,
      LoggerInterface::Ptr logger = nullptr);

  /// Process a single byte from the data stream
  /// @param byte The byte to process
  void processByte(std::uint8_t byte);

  /// Reset the decoder state
  void reset();

  /// Get the current mode
  Mode getCurrentMode() const { return current_mode_; }

private:
  /// Callback for sync frame detection
  void syncFrameDetectedCallback();

  /// Callback for data frame processing
  /// @param good_sync Indicates if synchronization is good
  /// @param frame_data The decoded frame data
  void dataframeCallback(bool good_sync, const DataFrameContents &frame_data);

  /// Callback for complete sweep data
  /// @param sweep_contents The sweep data
  void sweepCallback(const SweepBlockRawData &sweep_contents);

  /// Callback for bearing measurements
  /// @param sensor_bearings The bearing measurements
  void measurementCallback(const SweepBlockBearings &sensor_bearings);

  /// Current operating mode
  Mode current_mode_;

  /// Sync frame decoder (used in SYNC mode)
  std::unique_ptr<SyncFrameDecoder> sync_frame_decoder_;

  /// Data frame decoder (used in DATA mode)
  std::unique_ptr<DataFrameDecoder> data_frame_decoder_;

  /// Sweep processor
  std::unique_ptr<SweepProcessor> sweep_processor_;

  /// Measurement processor
  std::unique_ptr<MeasurementProcessor> measurement_processor_;

  /// OOTX decoders (one per base station, created on demand)
  std::array<std::unique_ptr<OOTXFrameDecoder>, kDeckLighthouseMaxNBs>
      ootx_decoders_;

  /// Previous timestamp0 values for each base station (for slow bit filtering)
  std::array<std::uint32_t, kDeckLighthouseMaxNBs> prev_timestamp0_;

  /// Application callback for bearing measurements
  BearingCallback app_bearing_callback_;

  /// Logger instance
  LoggerInterface::Ptr logger_;
};

} // namespace lighthouse_protocol_decoder

#endif // LIGHTHOUSE_PROTOCOL_DECODER__LIGHTHOUSE_PROTOCOL_DECODER_HPP_
