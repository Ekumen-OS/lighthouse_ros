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

#include "lighthouse_protocol_decoder/data_frame_decoder.hpp"

#include <algorithm>
#include <cassert>

namespace lighthouse_protocol_decoder
{

DataFrameDecoder::DataFrameDecoder(
  DataFrameCallback data_callback,
  LoggerInterface::Ptr logger)
: good_sync_(true), data_callback_(std::move(data_callback)),
  logger_(std::move(logger))
{
  frame_buffer_.reserve(12);
}

void DataFrameDecoder::processByte(std::uint8_t byte)
{
  frame_buffer_.push_back(byte);

  if (frame_buffer_.size() == 12) {
    // Check if this is a sync frame (all 0xFF) and if so ignore it
    const bool is_sync = std::all_of(
      //
      frame_buffer_.begin(),                       //
      frame_buffer_.end(),                         //
      [](std::uint8_t b) {return b == 0xFF;});     //

    if (!is_sync) {
      // Decode the data frame
      DataFrameContents frame_data = decodeFrame(frame_buffer_);

      // Validate padding fields
      if (frame_data.padding_1 != 0 || frame_data.padding_2 != 0) {
        if (logger_) {
          logger_->warning("Error: Bad padding value, resetting sync...");
        }
        good_sync_ = false;
      }

      // Invoke callback with decoded frame
      if (data_callback_) {
        data_callback_(good_sync_, frame_data);
      }
    }

    // Clear buffer for next frame
    frame_buffer_.clear();
  }
}

void DataFrameDecoder::reset()
{
  frame_buffer_.clear();
  good_sync_ = true;
}

std::uint32_t
DataFrameDecoder::readField(
  const std::vector<std::uint8_t> & frame_buffer,
  std::size_t start_bit, std::size_t bit_width)
{
  std::uint32_t field_value = 0;

  for (auto i = 0u; i < bit_width; ++i) {
    const auto current_bit = start_bit + i;
    const auto n_byte = current_bit / 8;
    const auto n_bit = current_bit % 8;

    const auto bit =
      static_cast<std::uint32_t>((frame_buffer[n_byte] >> n_bit) & 1);
    field_value |= (bit << i);
  }

  return field_value;
}

DataFrameContents
DataFrameDecoder::decodeFrame(const std::vector<std::uint8_t> & frame_buffer)
{
  assert(frame_buffer.size() == 12);

  DataFrameContents frame;

  frame.sid = static_cast<std::uint8_t>(readField(frame_buffer, 0 + 0, 2));
  frame.npoly = static_cast<std::uint8_t>(readField(frame_buffer, 0 + 2, 6));
  frame.width = static_cast<std::uint16_t>(readField(frame_buffer, 0 + 8, 16));
  frame.sync_offset = readField(frame_buffer, 24 + 0, 17);
  frame.padding_1 =
    static_cast<std::uint8_t>(readField(frame_buffer, 24 + 17, 7));
  frame.beam_word = readField(frame_buffer, 48 + 0, 17);
  frame.padding_2 =
    static_cast<std::uint8_t>(readField(frame_buffer, 48 + 17, 7));
  frame.timestamp = readField(frame_buffer, 72 + 0, 24);

  // Offset is expressed in a 6 MHz clock, while the timestamp uses a 24 MHz
  // clock. Convert offset to 24 MHz clock
  frame.sync_offset *= 4;

  frame.raw_data = frame_buffer;

  return frame;
}

}    // namespace lighthouse_protocol_decoder
