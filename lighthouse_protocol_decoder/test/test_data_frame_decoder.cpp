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

#include "lighthouse_protocol_decoder/data_frame_decoder.hpp"
#include "lighthouse_protocol_decoder/logger.hpp"
#include "test_helpers.hpp"

namespace lighthouse_protocol_decoder
{

using test_helpers::createDataFrame;

class DataFrameDecoderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    decoded_frames_.clear();
    good_sync_flags_.clear();

    decoder_ = std::make_unique<DataFrameDecoder>(
      [this](bool good_sync, const DataFrameContents & frame) {
        good_sync_flags_.push_back(good_sync);
        decoded_frames_.push_back(frame);
      },
      nullptr);
  }

  std::unique_ptr<DataFrameDecoder> decoder_;
  std::vector<DataFrameContents> decoded_frames_;
  std::vector<bool> good_sync_flags_;
};

TEST_F(DataFrameDecoderTest, InitialState) {
  EXPECT_TRUE(decoder_->hasGoodSync());
  EXPECT_EQ(decoded_frames_.size(), 0);
}

TEST_F(DataFrameDecoderTest, ConstructorWorksWithoutLogger) {
  // Verify constructor works with nullptr logger (default)
  std::vector<DataFrameContents> frames;
  auto decoder_no_logger = std::make_unique<DataFrameDecoder>(
    [&frames](bool, const DataFrameContents & frame) {
      frames.push_back(frame);
    });
  EXPECT_TRUE(decoder_no_logger->hasGoodSync());
}

TEST_F(DataFrameDecoderTest, NoCallbackWithFewerThan12Bytes) {
  // Send 11 bytes - should not trigger callback
  for (auto i = 0; i < 11; ++i) {
    decoder_->processByte(0x00);
  }

  EXPECT_EQ(decoded_frames_.size(), 0);
}

TEST_F(DataFrameDecoderTest, IgnoresSyncFrames) {
  // Send 12 bytes of 0xFF (sync frame) - should be ignored
  for (auto i = 0; i < 12; ++i) {
    decoder_->processByte(0xFF);
  }

  EXPECT_EQ(decoded_frames_.size(), 0);
}

TEST_F(DataFrameDecoderTest, DecodesValidDataFrame) {
  // Create a simple valid data frame with all fields set to 0
  auto frame = createDataFrame(0, 0, 0, 0, 0, 0, 0, 0);

  for (auto byte : frame) {
    decoder_->processByte(byte);
  }

  ASSERT_EQ(decoded_frames_.size(), 1);
  EXPECT_TRUE(good_sync_flags_[0]);
  EXPECT_EQ(decoded_frames_[0].sid, 0);
  EXPECT_EQ(decoded_frames_[0].npoly, 0);
  EXPECT_EQ(decoded_frames_[0].width, 0);
  EXPECT_EQ(decoded_frames_[0].sync_offset, 0);
  EXPECT_EQ(decoded_frames_[0].padding_1, 0);
  EXPECT_EQ(decoded_frames_[0].beam_word, 0);
  EXPECT_EQ(decoded_frames_[0].padding_2, 0);
  EXPECT_EQ(decoded_frames_[0].timestamp, 0);
}

TEST_F(DataFrameDecoderTest, BadPaddingSetsBadSync) {
  // Create a frame with non-zero padding_1 (invalid)
  auto frame = createDataFrame(0, 0, 0, 0, 1, 0, 0, 0);

  for (auto byte : frame) {
    decoder_->processByte(byte);
  }

  ASSERT_EQ(decoded_frames_.size(), 1);
  EXPECT_FALSE(good_sync_flags_[0]);
  EXPECT_FALSE(decoder_->hasGoodSync());
}

TEST_F(DataFrameDecoderTest, DecodesSidField) {
  // Test sid field with value 3 (maximum for 2 bits)
  auto frame = createDataFrame(3, 0, 0, 0, 0, 0, 0, 0);

  for (auto byte : frame) {
    decoder_->processByte(byte);
  }

  ASSERT_EQ(decoded_frames_.size(), 1);
  EXPECT_EQ(decoded_frames_[0].sid, 3);
}

TEST_F(DataFrameDecoderTest, DecodesNpolyField) {
  // Test npoly field with value 0x3F (maximum for 6 bits)
  auto frame = createDataFrame(0, 0x3F, 0, 0, 0, 0, 0, 0);

  for (auto byte : frame) {
    decoder_->processByte(byte);
  }

  ASSERT_EQ(decoded_frames_.size(), 1);
  EXPECT_EQ(decoded_frames_[0].npoly, 0x3F);
}

TEST_F(DataFrameDecoderTest, DecodesWidthField) {
  // Test width field with value 0x1234
  auto frame = createDataFrame(0, 0, 0x1234, 0, 0, 0, 0, 0);

  for (auto byte : frame) {
    decoder_->processByte(byte);
  }

  ASSERT_EQ(decoded_frames_.size(), 1);
  EXPECT_EQ(decoded_frames_[0].width, 0x1234);
}

TEST_F(DataFrameDecoderTest, DecodesSyncOffsetField) {
  // Test sync_offset field - raw value is multiplied by 4 by the decoder
  // Testing with max 17-bit value (0x1FFFF)
  auto frame = createDataFrame(0, 0, 0, 0x1FFFF, 0, 0, 0, 0);

  for (auto byte : frame) {
    decoder_->processByte(byte);
  }

  ASSERT_EQ(decoded_frames_.size(), 1);
  // 0x1FFFF * 4 = 0x7FFFC
  EXPECT_EQ(decoded_frames_[0].sync_offset, 0x1FFFF * 4);
}

TEST_F(DataFrameDecoderTest, DecodesBeamWordField) {
  // Test beam_word field with value 0x12345 (17-bit value)
  auto frame = createDataFrame(0, 0, 0, 0, 0, 0x12345, 0, 0);

  for (auto byte : frame) {
    decoder_->processByte(byte);
  }

  ASSERT_EQ(decoded_frames_.size(), 1);
  EXPECT_EQ(decoded_frames_[0].beam_word, 0x12345);
}

TEST_F(DataFrameDecoderTest, DecodesTimestampField) {
  // Test timestamp field with value 0x345678 (24-bit value)
  auto frame = createDataFrame(0, 0, 0, 0, 0, 0, 0, 0x345678);

  for (auto byte : frame) {
    decoder_->processByte(byte);
  }

  ASSERT_EQ(decoded_frames_.size(), 1);
  EXPECT_EQ(decoded_frames_[0].timestamp, 0x345678);
}

TEST_F(DataFrameDecoderTest, DecodesAllFieldsTogether) {
  // Test all fields with non-zero values to ensure proper bit packing
  auto frame = createDataFrame(
    2,                                  // sid (2 bits)
    0x2A,                               // npoly (6 bits)
    0xABCD,                             // width (16 bits)
    0x15678,                            // sync_offset (17 bits)
    0,                                  // padding_1 (must be 0 for valid frame)
    0x1BCDE,                            // beam_word (17 bits)
    0,                                  // padding_2 (must be 0 for valid frame)
    0xFEDCBA                            // timestamp (24 bits)
  );

  for (auto byte : frame) {
    decoder_->processByte(byte);
  }

  ASSERT_EQ(decoded_frames_.size(), 1);
  EXPECT_TRUE(good_sync_flags_[0]);
  EXPECT_EQ(decoded_frames_[0].sid, 2);
  EXPECT_EQ(decoded_frames_[0].npoly, 0x2A);
  EXPECT_EQ(decoded_frames_[0].width, 0xABCD);
  EXPECT_EQ(decoded_frames_[0].sync_offset, 0x15678 * 4);
  EXPECT_EQ(decoded_frames_[0].padding_1, 0);
  EXPECT_EQ(decoded_frames_[0].beam_word, 0x1BCDE);
  EXPECT_EQ(decoded_frames_[0].padding_2, 0);
  EXPECT_EQ(decoded_frames_[0].timestamp, 0xFEDCBA);
}

TEST_F(DataFrameDecoderTest, StoresRawData) {
  std::vector<std::uint8_t> frame = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
    0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C};

  for (auto byte : frame) {
    decoder_->processByte(byte);
  }

  ASSERT_EQ(decoded_frames_.size(), 1);
  ASSERT_EQ(decoded_frames_[0].raw_data.size(), 12);
  for (auto i = 0u; i < 12; ++i) {
    EXPECT_EQ(decoded_frames_[0].raw_data[i], frame[i]);
  }
}

TEST_F(DataFrameDecoderTest, MultipleFrames) {
  // Send two valid frames with different sid values
  auto frame1 = createDataFrame(0, 0, 0, 0, 0, 0, 0, 0);
  auto frame2 = createDataFrame(1, 0, 0, 0, 0, 0, 0, 0);

  for (auto byte : frame1) {
    decoder_->processByte(byte);
  }
  for (auto byte : frame2) {
    decoder_->processByte(byte);
  }

  ASSERT_EQ(decoded_frames_.size(), 2);
  EXPECT_EQ(decoded_frames_[0].sid, 0);
  EXPECT_EQ(decoded_frames_[1].sid, 1);
}

TEST_F(DataFrameDecoderTest, ResetClearsBuffer) {
  // Send 11 bytes
  for (auto i = 0; i < 11; ++i) {
    decoder_->processByte(0x00);
  }

  // Reset
  decoder_->reset();

  // Send 1 byte - should not trigger callback
  decoder_->processByte(0x00);
  EXPECT_EQ(decoded_frames_.size(), 0);

  // Send 12 bytes - should trigger callback
  for (auto i = 0; i < 12; ++i) {
    decoder_->processByte(0x00);
  }
  EXPECT_EQ(decoded_frames_.size(), 1);
}

TEST_F(DataFrameDecoderTest, ResetRestoresGoodSync) {
  // Create frame with bad padding
  auto bad_frame = createDataFrame(0, 0, 0, 0, 1, 0, 0, 0);

  for (auto byte : bad_frame) {
    decoder_->processByte(byte);
  }

  EXPECT_FALSE(decoder_->hasGoodSync());

  // Reset should restore good sync
  decoder_->reset();
  EXPECT_TRUE(decoder_->hasGoodSync());
}

TEST_F(DataFrameDecoderTest, SyncFrameDoesNotAffectState) {
  // Send a valid data frame
  auto frame = createDataFrame(0, 0, 0, 0, 0, 0, 0, 0);
  for (auto byte : frame) {
    decoder_->processByte(byte);
  }

  EXPECT_EQ(decoded_frames_.size(), 1);

  // Send a sync frame (should be ignored)
  for (auto i = 0; i < 12; ++i) {
    decoder_->processByte(0xFF);
  }

  // Should still have only 1 decoded frame
  EXPECT_EQ(decoded_frames_.size(), 1);
}

TEST_F(DataFrameDecoderTest, BufferClearsAfterFrame) {
  // Send 12 bytes
  for (auto i = 0; i < 12; ++i) {
    decoder_->processByte(0x00);
  }

  EXPECT_EQ(decoded_frames_.size(), 1);

  // Send 11 more bytes - should not trigger callback yet
  for (auto i = 0; i < 11; ++i) {
    decoder_->processByte(0x00);
  }

  EXPECT_EQ(decoded_frames_.size(), 1);

  // Send 1 more byte - should trigger second callback
  decoder_->processByte(0x00);
  EXPECT_EQ(decoded_frames_.size(), 2);
}

}    // namespace lighthouse_protocol_decoder
