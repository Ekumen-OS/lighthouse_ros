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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "lighthouse_protocol_decoder/logger.hpp"
#include "lighthouse_protocol_decoder/sync_frame_decoder.hpp"

namespace lighthouse_protocol_decoder
{

class SyncFrameDecoderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    sync_detected_count_ = 0;

    decoder_ = std::make_unique<SyncFrameDecoder>(
      [this]() {sync_detected_count_++;}, nullptr);
  }

  std::unique_ptr<SyncFrameDecoder> decoder_;
  int sync_detected_count_;
};

TEST_F(SyncFrameDecoderTest, ConstructorWorksWithoutLogger) {
  // Verify constructor works with nullptr logger (default)
  int count = 0;
  auto decoder_no_logger =
    std::make_unique<SyncFrameDecoder>([&count]() {count++;});
  // Send 12 bytes of 0xFF to trigger sync
  for (auto i = 0; i < 12; ++i) {
    decoder_no_logger->processByte(0xFF);
  }
  EXPECT_EQ(count, 1);
}

TEST_F(SyncFrameDecoderTest, NoSyncDetectedWithFewerThan12Bytes) {
  // Send 11 bytes of 0xFF - should not trigger sync
  for (auto i = 0; i < 11; ++i) {
    decoder_->processByte(0xFF);
  }

  EXPECT_EQ(sync_detected_count_, 0);
}

TEST_F(SyncFrameDecoderTest, SyncDetectedWith12BytesOfFF) {
  // Send exactly 12 bytes of 0xFF - should trigger sync once
  for (auto i = 0; i < 12; ++i) {
    decoder_->processByte(0xFF);
  }

  EXPECT_EQ(sync_detected_count_, 1);
}

TEST_F(SyncFrameDecoderTest, SyncDetectedOnlyOnceFor12Bytes) {
  // Send 12 bytes of 0xFF - should trigger sync exactly once
  for (auto i = 0; i < 12; ++i) {
    decoder_->processByte(0xFF);
  }

  EXPECT_EQ(sync_detected_count_, 1);

  // Send another byte - this creates a rolling window with 12 bytes of 0xFF
  // again
  decoder_->processByte(0xFF);

  EXPECT_EQ(sync_detected_count_, 2);
}

TEST_F(SyncFrameDecoderTest, NoSyncWithMixedBytes) {
  // Send 12 bytes with one non-0xFF byte - should not trigger sync
  for (auto i = 0; i < 11; ++i) {
    decoder_->processByte(0xFF);
  }
  decoder_->processByte(0xFE);  // Different byte

  EXPECT_EQ(sync_detected_count_, 0);
}

TEST_F(SyncFrameDecoderTest, SyncAfterGarbage) {
  // Send some garbage bytes first
  decoder_->processByte(0x12);
  decoder_->processByte(0x34);
  decoder_->processByte(0x56);

  // Now send 12 bytes of 0xFF - should trigger sync
  for (auto i = 0; i < 12; ++i) {
    decoder_->processByte(0xFF);
  }

  EXPECT_EQ(sync_detected_count_, 1);
}

TEST_F(SyncFrameDecoderTest, RollingWindowBehavior) {
  // Send 11 bytes of 0xFF
  for (auto i = 0; i < 11; ++i) {
    decoder_->processByte(0xFF);
  }
  EXPECT_EQ(sync_detected_count_, 0);

  // Send one non-FF byte
  decoder_->processByte(0x00);
  EXPECT_EQ(sync_detected_count_, 0);

  // Send 12 bytes of 0xFF - should trigger sync
  for (auto i = 0; i < 12; ++i) {
    decoder_->processByte(0xFF);
  }
  EXPECT_EQ(sync_detected_count_, 1);
}

TEST_F(SyncFrameDecoderTest, ResetClearsBuffer) {
  // Send 11 bytes of 0xFF
  for (auto i = 0; i < 11; ++i) {
    decoder_->processByte(0xFF);
  }

  // Reset
  decoder_->reset();

  // Send one more byte - should not trigger sync because buffer was cleared
  decoder_->processByte(0xFF);
  EXPECT_EQ(sync_detected_count_, 0);

  // Now send 11 bytes of 0xFF - should trigger sync once
  for (auto i = 0; i < 11; ++i) {
    decoder_->processByte(0xFF);
  }
  EXPECT_EQ(sync_detected_count_, 1);
}

TEST_F(SyncFrameDecoderTest, MultipleSyncsDetected) {
  // Send 12 bytes of 0xFF - first sync
  for (auto i = 0; i < 12; ++i) {
    decoder_->processByte(0xFF);
  }
  EXPECT_EQ(sync_detected_count_, 1);

  // Send some data bytes
  for (auto i = 0; i < 5; ++i) {
    decoder_->processByte(0x00);
  }

  // Send another 12 bytes of 0xFF - second sync
  for (auto i = 0; i < 12; ++i) {
    decoder_->processByte(0xFF);
  }
  EXPECT_EQ(sync_detected_count_, 2);
}

TEST_F(SyncFrameDecoderTest, ContinuousFFStream) {
  // Send 24 bytes of 0xFF - should trigger multiple syncs
  // At byte 12: first sync
  // At byte 13: second sync (rolling window)
  // etc.
  for (auto i = 0; i < 24; ++i) {
    decoder_->processByte(0xFF);
  }

  // Should have detected 13 syncs (at bytes 12, 13, 14, ..., 24)
  EXPECT_EQ(sync_detected_count_, 13);
}

}    // namespace lighthouse_protocol_decoder
