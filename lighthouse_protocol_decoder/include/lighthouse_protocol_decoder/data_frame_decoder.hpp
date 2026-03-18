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

#ifndef LIGHTHOUSE_PROTOCOL_DECODER__DATA_FRAME_DECODER_HPP_
#define LIGHTHOUSE_PROTOCOL_DECODER__DATA_FRAME_DECODER_HPP_

#include <cstdint>
#include <memory>
#include <vector>

#include "lighthouse_protocol_decoder/datatypes.hpp"
#include "lighthouse_protocol_decoder/logger.hpp"

namespace lighthouse_protocol_decoder {

/// Decoder for data frames (12 bytes containing sensor measurements)
class DataFrameDecoder {
public:
  /// Constructor
  /// @param data_callback Callback invoked when data frame is decoded
  /// @param logger Logger instance for debug/warning messages (optional)
  explicit DataFrameDecoder(DataFrameCallback data_callback,
                            LoggerInterface::Ptr logger = nullptr);

  /// Process a single byte from the data stream
  /// @param byte The byte to process
  void processByte(std::uint8_t byte);

  /// Reset the decoder state
  void reset();

  /// Check if synchronization is good
  bool hasGoodSync() const { return good_sync_; }

private:
  /// Read a multi-bit field from the frame buffer
  /// @param frame_buffer The frame buffer to read from
  /// @param start_bit Starting bit position
  /// @param bit_width Number of bits to read
  /// @return The field value
  static std::uint32_t readField(const std::vector<std::uint8_t> &frame_buffer,
                                 std::size_t start_bit, std::size_t bit_width);

  /// Decode a complete 12-byte frame
  /// @param frame_buffer The frame buffer to decode
  /// @return The decoded frame contents
  static DataFrameContents
  decodeFrame(const std::vector<std::uint8_t> &frame_buffer);

  /// Buffer for accumulating bytes (exactly 12 bytes)
  std::vector<std::uint8_t> frame_buffer_;

  /// Flag indicating if synchronization is good
  bool good_sync_;

  /// Callback for decoded data frames
  DataFrameCallback data_callback_;

  /// Logger instance
  LoggerInterface::Ptr logger_;
};

} // namespace lighthouse_protocol_decoder

#endif // LIGHTHOUSE_PROTOCOL_DECODER__DATA_FRAME_DECODER_HPP_
