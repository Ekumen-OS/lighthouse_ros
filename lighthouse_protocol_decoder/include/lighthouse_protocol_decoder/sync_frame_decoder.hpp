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

#ifndef LIGHTHOUSE_PROTOCOL_DECODER__SYNC_FRAME_DECODER_HPP_
#define LIGHTHOUSE_PROTOCOL_DECODER__SYNC_FRAME_DECODER_HPP_

#include <cstdint>
#include <deque>
#include <memory>

#include "lighthouse_protocol_decoder/datatypes.hpp"
#include "lighthouse_protocol_decoder/logger.hpp"

namespace lighthouse_protocol_decoder {

/// Decoder for sync frames (12 bytes of 0xFF)
class SyncFrameDecoder {
public:
  /// Constructor
  /// @param sync_callback Callback invoked when sync frame is detected
  /// @param logger Logger instance for debug messages (optional)
  explicit SyncFrameDecoder(SyncFrameDetectedCallback sync_callback,
                            LoggerInterface::Ptr logger = nullptr);

  /// Process a single byte from the data stream
  /// @param byte The byte to process
  void processByte(std::uint8_t byte);

  /// Reset the decoder state
  void reset();

private:
  /// Buffer for accumulating bytes (rolling window of 12 bytes)
  std::deque<std::uint8_t> frame_buffer_;

  /// Callback for sync frame detection
  SyncFrameDetectedCallback sync_callback_;

  /// Logger instance
  LoggerInterface::Ptr logger_;
};

} // namespace lighthouse_protocol_decoder

#endif // LIGHTHOUSE_PROTOCOL_DECODER__SYNC_FRAME_DECODER_HPP_
