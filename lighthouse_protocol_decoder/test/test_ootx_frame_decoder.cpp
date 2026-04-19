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

#include "lighthouse_protocol_decoder/crc32.hpp"
#include "lighthouse_protocol_decoder/logger.hpp"
#include "lighthouse_protocol_decoder/ootx_frame_decoder.hpp"

namespace lighthouse_protocol_decoder
{

class OOTXFrameDecoderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    decoder_ = std::make_unique<OOTXFrameDecoder>(nullptr);
  }

  std::unique_ptr<OOTXFrameDecoder> decoder_;
};

TEST_F(OOTXFrameDecoderTest, InitialState) {
  EXPECT_FALSE(decoder_->hasDecodedFrame());
  EXPECT_TRUE(decoder_->getLastPayload().empty());
}

TEST_F(OOTXFrameDecoderTest, ConstructorWorksWithoutLogger) {
  // Verify constructor works with nullptr logger (default)
  auto decoder_no_logger = std::make_unique<OOTXFrameDecoder>();
  EXPECT_FALSE(decoder_no_logger->hasDecodedFrame());
}

TEST_F(OOTXFrameDecoderTest, ResetClearsState) {
  // Add some bits
  decoder_->processSlowBit(false);
  decoder_->processSlowBit(true);

  decoder_->reset();

  EXPECT_FALSE(decoder_->hasDecodedFrame());
  EXPECT_TRUE(decoder_->getLastPayload().empty());
}

TEST_F(OOTXFrameDecoderTest, ValidFrameDecoding) {
  // Create a minimal valid OOTX frame:
  // - Preamble: 16 zeros + sync bit (1)
  // - Length: 2 bytes (length=2 in little-endian: 0x0200) + sync bit
  // - Data: 2 bytes (0xAB, 0xCD) in one 16-bit word + sync bit
  // - CRC32: 2 words (4 bytes) with proper CRC + sync bits

  std::vector<std::uint8_t> payload = {0xAB, 0xCD};

  // Calculate expected CRC32 for this payload
  const std::uint32_t expected_crc = calculateCRC32(payload);

  // Helper to add a 16-bit word with sync bit
  auto add_word = [this](std::uint16_t word) {
      // MSB first
      for (auto i = 15; i >= 0; --i) {
        decoder_->processSlowBit((word >> i) & 1);
      }
      decoder_->processSlowBit(true);  // sync bit
    };

  // Helper to add a 16-bit word in little-endian byte order with sync bit
  auto add_word_le = [this](std::uint16_t word) {
      // Swap bytes then send MSB first
      const auto swapped = ((word & 0xFF) << 8) | ((word >> 8) & 0xFF);
      for (auto i = 15; i >= 0; --i) {
        decoder_->processSlowBit((swapped >> i) & 1);
      }
      decoder_->processSlowBit(true);  // sync bit
    };

  // 1. Preamble (16 zeros)
  add_word(0x0000);

  // 2. Length (2 in little-endian byte order)
  add_word_le(0x0002);

  // 3. Data (0xAB, 0xCD as little-endian word: 0xCDAB)
  add_word(0xCDAB);

  // 4. CRC32 lower 16 bits
  add_word_le(expected_crc & 0xFFFF);

  // 5. CRC32 upper 16 bits
  add_word_le((expected_crc >> 16) & 0xFFFF);

  // Verify decoding
  EXPECT_TRUE(decoder_->hasDecodedFrame());
  EXPECT_EQ(decoder_->getLastPayload(), payload);
}

TEST_F(OOTXFrameDecoderTest, InvalidPreambleRejected) {
  // Send a word that's not all zeros for preamble
  for (int i = 0; i < 16; ++i) {
    decoder_->processSlowBit(i == 0);  // First bit is 1, rest are 0
  }
  decoder_->processSlowBit(true);  // sync bit

  // Send length
  for (int i = 0; i < 16; ++i) {
    decoder_->processSlowBit(false);
  }
  decoder_->processSlowBit(true);

  // Should not have decoded
  EXPECT_FALSE(decoder_->hasDecodedFrame());
}

TEST_F(OOTXFrameDecoderTest, InvalidSyncBitRejected) {
  // Preamble with invalid sync bit
  for (int i = 0; i < 16; ++i) {
    decoder_->processSlowBit(false);
  }
  decoder_->processSlowBit(false);  // Wrong! Should be 1

  // Should not decode
  EXPECT_FALSE(decoder_->hasDecodedFrame());
}

TEST_F(OOTXFrameDecoderTest, ExcessiveLengthRejected) {
  // Valid preamble
  for (int i = 0; i < 16; ++i) {
    decoder_->processSlowBit(false);
  }
  decoder_->processSlowBit(true);

  // Length > 43 (send 50 in little-endian: 0x3200)
  std::uint16_t invalid_length = 50;
  std::uint16_t le_length =
    ((invalid_length & 0xFF) << 8) | ((invalid_length >> 8) & 0xFF);
  for (int i = 15; i >= 0; --i) {
    decoder_->processSlowBit((le_length >> i) & 1);
  }
  decoder_->processSlowBit(true);

  EXPECT_FALSE(decoder_->hasDecodedFrame());
}

TEST_F(OOTXFrameDecoderTest, CRCMismatchRejected) {
  // Similar to valid frame test but with wrong CRC
  auto add_word = [this](std::uint16_t word) {
      for (int i = 15; i >= 0; --i) {
        decoder_->processSlowBit((word >> i) & 1);
      }
      decoder_->processSlowBit(true);
    };

  auto add_word_le = [this](std::uint16_t word) {
      std::uint16_t swapped = ((word & 0xFF) << 8) | ((word >> 8) & 0xFF);
      for (int i = 15; i >= 0; --i) {
        decoder_->processSlowBit((swapped >> i) & 1);
      }
      decoder_->processSlowBit(true);
    };

  add_word(0x0000);    // Preamble
  add_word_le(0x0002);  // Length = 2
  add_word(0xCDAB);    // Data
  add_word_le(0x1234);  // Wrong CRC lower
  add_word_le(0x5678);  // Wrong CRC upper

  EXPECT_FALSE(decoder_->hasDecodedFrame());
}

TEST_F(OOTXFrameDecoderTest, ValidFrameDecodedAfterCRCMismatch) {
  // Verify that after a CRC failure the decoder recovers and can decode
  // a subsequent valid frame without getting stuck draining corrupted bits.

  auto add_word = [this](std::uint16_t word) {
      for (int i = 15; i >= 0; --i) {
        decoder_->processSlowBit((word >> i) & 1);
      }
      decoder_->processSlowBit(true);
    };

  auto add_word_le = [this](std::uint16_t word) {
      std::uint16_t swapped = ((word & 0xFF) << 8) | ((word >> 8) & 0xFF);
      for (int i = 15; i >= 0; --i) {
        decoder_->processSlowBit((swapped >> i) & 1);
      }
      decoder_->processSlowBit(true);
    };

  // First frame: structurally valid but with a bad CRC
  add_word(0x0000);     // Preamble
  add_word_le(0x0002);  // Length = 2
  add_word(0xCDAB);     // Data
  add_word_le(0x1234);  // Wrong CRC lower
  add_word_le(0x5678);  // Wrong CRC upper

  EXPECT_FALSE(decoder_->hasDecodedFrame());

  // Second frame: fully valid — must be decoded successfully
  std::vector<std::uint8_t> payload = {0xAB, 0xCD};
  const std::uint32_t expected_crc = calculateCRC32(payload);

  add_word(0x0000);                             // Preamble
  add_word_le(0x0002);                          // Length = 2
  add_word(0xCDAB);                             // Data
  add_word_le(expected_crc & 0xFFFF);           // CRC lower
  add_word_le((expected_crc >> 16) & 0xFFFF);   // CRC upper

  EXPECT_TRUE(decoder_->hasDecodedFrame());
  EXPECT_EQ(decoder_->getLastPayload(), payload);
}

}    // namespace lighthouse_protocol_decoder

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
