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

#include "lighthouse_protocol_decoder/sync_frame_decoder.hpp"

#include <algorithm>

namespace lighthouse_protocol_decoder {

SyncFrameDecoder::SyncFrameDecoder(SyncFrameDetectedCallback sync_callback,
                                   LoggerInterface::Ptr logger)
    : sync_callback_(std::move(sync_callback)), logger_(std::move(logger)) {}

void SyncFrameDecoder::processByte(std::uint8_t byte) {
  frame_buffer_.push_back(byte);

  // Maintain a rolling window of 12 bytes
  while (frame_buffer_.size() > 12) {
    frame_buffer_.pop_front();
  }

  if (frame_buffer_.size() < 12) {
    return;
  }

  // Check if all 12 bytes are 0xFF (sync frame)
  const bool is_sync = std::all_of(              //
      frame_buffer_.begin(),                     //
      frame_buffer_.end(),                       //
      [](std::uint8_t b) { return b == 0xFF; }); //

  if (is_sync && sync_callback_) {
    sync_callback_();
  }
}

void SyncFrameDecoder::reset() { frame_buffer_.clear(); }

} // namespace lighthouse_protocol_decoder
