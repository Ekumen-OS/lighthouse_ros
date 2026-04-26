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

#include "lighthouse_protocol_decoder/ootx_frame_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "lighthouse_protocol_decoder/crc32.hpp"

namespace lighthouse_protocol_decoder
{

namespace
{
/// Convert a 16-bit value from line byte order (big-endian) to local byte order
/// (little-endian)
/// @param value The value in line byte order
/// @return The value in local byte order
std::uint16_t lineToLocalByteOrder(std::uint16_t value)
{
  return (value << 8) | (value >> 8);
}
}    // namespace

OOTXFrameDecoder::OOTXFrameDecoder(LoggerInterface::Ptr logger)
: has_decoded_frame_(false),
  logger_(logger ? std::move(logger) : std::make_shared<NullLogger>()) {}

void OOTXFrameDecoder::processSlowBit(bool slow_bit)
{
  bit_buffer_.push_back(slow_bit);

  while (!bit_buffer_.empty()) {
    const auto result = tryDecodeFrame();

    if (result == DecodeResult::Incomplete) {
      // Need more bits, break and wait for new data
      break;
    } else if (result == DecodeResult::NotAFrame) {
      // Remove bits from the buffer until a 0 comes out. This is because
      // the true preamble is 17 0s followed by a one, but our matcher
      // can only search for 16 0s followed by a one. We remove bits until we
      // find a candidate to be the 17th 0.
      while (!bit_buffer_.empty() && bit_buffer_.front()) {
        bit_buffer_.pop_front();
      }
      if (!bit_buffer_.empty()) {
        bit_buffer_.pop_front();  // Remove the first 0 we found
      }
    } else if (result == DecodeResult::Decoded) {
      // Successfully decoded, clear the buffer
      bit_buffer_.clear();
    }
  }
}

DecodeResult OOTXFrameDecoder::tryDecodeFrame()
{
  // Buffer payload_length must be a multiple of 17
  if (bit_buffer_.size() % 17 != 0) {
    return DecodeResult::Incomplete;
  }

  const auto words_count = bit_buffer_.size() / 17;

  // Need at least 2 words (preamble + payload_length)
  if (words_count < 2) {
    return DecodeResult::Incomplete;
  }

  // Check that every 17th bit (sync bit) is a 1
  for (auto i = 0u; i < words_count; ++i) {
    if (!bit_buffer_[i * 17 + 16]) {
      return DecodeResult::NotAFrame;
    }
  }

  // First word (data bits, not sync) must be all zeros (preamble)
  const auto preamble = loadUint16MSBFirst(bit_buffer_, 0, 16);
  if (preamble != 0) {
    return DecodeResult::NotAFrame;
  }

  // Second word is the payload_length (in little-endian byte order)
  const auto length_be = loadUint16MSBFirst(bit_buffer_, 17, 16);
  const auto payload_length = lineToLocalByteOrder(length_be);

  if (payload_length > kOOTXMaxPayloadLength) {
    return DecodeResult::NotAFrame;
  }

  // Calculate expected word count:
  // preamble(1) + payload_length(1) + data + crc32(2)
  const auto data_words =
    (payload_length + 1) / 2;   // round the payload len up to nearest word
  const std::size_t expected_words = 2 + data_words + 2;

  if (words_count == 2) {
    logger_->debug("OOTX preamble detected, accumulating payload...");
  }

  {
    std::ostringstream oss;
    oss << "OOTX progress: " << words_count << " / " << expected_words << " words received";
    logger_->debug(oss.str());
  }

  // Wait until we have all the bits
  if (words_count < expected_words) {
    return DecodeResult::Incomplete;
  }

  // Extract data words (skip preamble and payload_length, exclude last 2 CRC
  // words)
  std::vector<std::uint8_t> payload;
  payload.reserve(payload_length);

  for (auto word_idx = 2u; word_idx < expected_words - 2; ++word_idx) {
    const auto bit_offset = word_idx * 17;
    const auto word = loadUint16MSBFirst(bit_buffer_, bit_offset, 16);

    // Convert word to bytes (little-endian)
    payload.push_back(static_cast<std::uint8_t>(word & 0xFF));
    if (payload.size() < payload_length) {
      payload.push_back(static_cast<std::uint8_t>((word >> 8) & 0xFF));
    }
  }

  // Truncate to actual payload_length (removes padding if payload_length is
  // odd)
  payload.resize(payload_length);

  // Extract CRC32 (last two words, in little-endian byte order)
  const std::size_t crc_lower_offset = (expected_words - 2) * 17;
  const std::size_t crc_upper_offset = (expected_words - 1) * 17;

  const std::uint16_t crc_lower_be =
    loadUint16MSBFirst(bit_buffer_, crc_lower_offset, 16);
  const std::uint16_t crc_upper_be =
    loadUint16MSBFirst(bit_buffer_, crc_upper_offset, 16);

  const std::uint16_t crc_lower = lineToLocalByteOrder(crc_lower_be);
  const std::uint16_t crc_upper = lineToLocalByteOrder(crc_upper_be);

  const std::uint32_t transmitted_crc =
    (static_cast<std::uint32_t>(crc_upper) << 16) | crc_lower;
  const std::uint32_t calculated_crc = calculateCRC32(payload);

  if (calculated_crc != transmitted_crc) {
    logger_->warning("OOTX frame CRC32 mismatch, discarding frame.");
    return DecodeResult::NotAFrame;
  }

  // Successfully decoded!
  latest_payload_ = std::move(payload);
  has_decoded_frame_ = true;

  logger_->info("OOTX corrections package successfully decoded.");

  return DecodeResult::Decoded;
}

std::uint16_t OOTXFrameDecoder::loadUint16MSBFirst(
  const std::deque<bool> & bits,
  std::size_t offset,
  std::size_t count)
{
  std::uint16_t value = 0;
  for (auto i = 0u; i < count; ++i) {
    if (bits[offset + count - 1 - i]) {
      value |= (1u << i);
    }
  }
  return value;
}

void OOTXFrameDecoder::reset()
{
  bit_buffer_.clear();
  latest_payload_.clear();
  has_decoded_frame_ = false;
}

}    // namespace lighthouse_protocol_decoder
