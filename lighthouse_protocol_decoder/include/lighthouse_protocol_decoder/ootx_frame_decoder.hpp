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

#ifndef LIGHTHOUSE_PROTOCOL_DECODER__OOTX_FRAME_DECODER_HPP_
#define LIGHTHOUSE_PROTOCOL_DECODER__OOTX_FRAME_DECODER_HPP_

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lighthouse_protocol_decoder/datatypes.hpp"
#include "lighthouse_protocol_decoder/logger.hpp"

namespace lighthouse_protocol_decoder
{

/// Maximum payload length for OOTX frames
constexpr std::size_t kOOTXMaxPayloadLength = 43;

/// Result of attempting to decode a frame from the buffer
enum class DecodeResult
{
  /// Buffer does not contain a valid frame, bits should be discarded
  NotAFrame,
  /// Frame successfully decoded
  Decoded,
  /// Not enough bits to determine if frame is valid, need more data
  Incomplete
};

/// Decoder for the OOTX (Omnidirectional Optical Transmitter) protocol
/// used by Lighthouse base stations to transmit calibration data.
class OOTXFrameDecoder
{
public:
  /// Constructor
  /// @param logger Logger instance for debug/info/warning messages (optional)
  explicit OOTXFrameDecoder(LoggerInterface::Ptr logger = nullptr);

  /// Process a single slow bit from the Lighthouse data stream
  /// @param slow_bit The bit value (0 or 1)
  void processSlowBit(bool slow_bit);

  /// Get the last successfully decoded payload
  /// @return The payload bytes, or empty if no frame has been decoded yet
  const std::vector<std::uint8_t> & getLastPayload() const
  {
    return latest_payload_;
  }

  /// Check if a frame has been successfully decoded
  bool hasDecodedFrame() const {return has_decoded_frame_;}

  /// Reset the decoder state
  void reset();

private:
  /// Try to decode a frame from the current bit buffer
  /// @return The result of the decode attempt
  DecodeResult tryDecodeFrame();

  /// Convert a sequence of bits to an unsigned integer (MSB first)
  /// @param bits The bit sequence
  /// @param offset Starting position in the bit sequence
  /// @param count Number of bits to read (default 16)
  /// @return The integer value
  static std::uint16_t loadUint16MSBFirst(
    const std::deque<bool> & bits,
    std::size_t offset,
    std::size_t count = 16);

  /// Extract a 16-bit word from the bit buffer
  /// @param word_index The index of the word to extract (0-based)
  /// @return The 16-bit word value
  std::optional<std::uint16_t> extractWord(std::size_t word_index) const;

  /// Bit buffer for accumulating incoming slow bits
  std::deque<bool> bit_buffer_;

  /// Last successfully decoded payload
  std::vector<std::uint8_t> latest_payload_;

  /// Flag indicating if a frame has been decoded
  bool has_decoded_frame_;

  /// Logger instance
  LoggerInterface::Ptr logger_;
};

}    // namespace lighthouse_protocol_decoder

#endif  // LIGHTHOUSE_PROTOCOL_DECODER__OOTX_FRAME_DECODER_HPP_
