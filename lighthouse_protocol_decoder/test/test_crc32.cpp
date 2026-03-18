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

#include "lighthouse_protocol_decoder/crc32.hpp"

#include <string>
#include <vector>

namespace lighthouse_protocol_decoder {

class CRC32Test : public ::testing::Test {};

TEST_F(CRC32Test, EmptyData) {
  // CRC32 of empty data should be a specific known value
  std::vector<std::uint8_t> data;
  const auto crc = calculateCRC32(data);

  EXPECT_EQ(crc, 0x00000000);
}

TEST_F(CRC32Test, SingleByte) {
  // Test CRC32 with a single byte
  std::vector<std::uint8_t> data = {0x00};
  const auto crc = calculateCRC32(data);

  EXPECT_EQ(crc, 0xD202EF8D);
}

TEST_F(CRC32Test, SingleByteFF) {
  // Test CRC32 with a single 0xFF byte
  std::vector<std::uint8_t> data = {0xFF};
  const auto crc = calculateCRC32(data);

  EXPECT_EQ(crc, 0xFF000000);
}

TEST_F(CRC32Test, KnownASCIIString) {
  // Test with a known ASCII string "123456789"
  // This is a standard test vector for CRC32
  std::vector<std::uint8_t> data = {'1', '2', '3', '4', '5',
                                    '6', '7', '8', '9'};
  const std::uint32_t crc = calculateCRC32(data);

  EXPECT_EQ(crc, 0xCBF43926);
}

TEST_F(CRC32Test, AnotherKnownString) {
  // Test with "The quick brown fox jumps over the lazy dog"
  const std::string str = "The quick brown fox jumps over the lazy dog";
  const std::vector<std::uint8_t> data(str.begin(), str.end());
  const auto crc = calculateCRC32(data);

  EXPECT_EQ(crc, 0x414FA339);
}

TEST_F(CRC32Test, AllZeros) {
  // Test with multiple zero bytes
  const std::vector<std::uint8_t> data(10, 0x00);
  const auto crc = calculateCRC32(data);

  EXPECT_EQ(crc, 0xE38A6876);
}

TEST_F(CRC32Test, AllOnes) {
  // Test with multiple 0xFF bytes
  const std::vector<std::uint8_t> data(10, 0xFF);
  const auto crc = calculateCRC32(data);

  EXPECT_EQ(crc, 0x0FE4B35C);
}

TEST_F(CRC32Test, SequentialBytes) {
  // Test with sequential bytes 0-255
  std::vector<std::uint8_t> data(256);
  for (auto i = 0; i < 256; ++i) {
    data[i] = static_cast<std::uint8_t>(i);
  }
  const auto crc = calculateCRC32(data);

  EXPECT_EQ(crc, 0x29058C73);
}

TEST_F(CRC32Test, DifferentDataDifferentCRC) {
  // Ensure different data produces different CRC
  const std::vector<std::uint8_t> data1 = {0x01, 0x02, 0x03};
  const std::vector<std::uint8_t> data2 = {0x01, 0x02, 0x04};

  const auto crc1 = calculateCRC32(data1);
  const auto crc2 = calculateCRC32(data2);

  EXPECT_NE(crc1, crc2);
}

TEST_F(CRC32Test, OrderMatters) {
  // Ensure byte order matters
  const std::vector<std::uint8_t> data1 = {0x01, 0x02, 0x03};
  const std::vector<std::uint8_t> data2 = {0x03, 0x02, 0x01};

  const auto crc1 = calculateCRC32(data1);
  const auto crc2 = calculateCRC32(data2);

  EXPECT_NE(crc1, crc2);
}

TEST_F(CRC32Test, SameDataSameCRC) {
  // Ensure same data produces same CRC (deterministic)
  const std::vector<std::uint8_t> data = {0xAB, 0xCD, 0xEF, 0x12, 0x34};

  const auto crc1 = calculateCRC32(data);
  const auto crc2 = calculateCRC32(data);

  EXPECT_EQ(crc1, crc2);
}

TEST_F(CRC32Test, LargeData) {
  // Test with larger data set (1KB)
  std::vector<std::uint8_t> data(1024);
  for (auto i = 0u; i < data.size(); ++i) {
    data[i] = static_cast<std::uint8_t>(i & 0xFF);
  }

  const auto crc = calculateCRC32(data);
  // The exact value depends on the repeating pattern
  EXPECT_NE(crc, 0xFFFFFFFF); // Should not be the initial value
}

TEST_F(CRC32Test, TwoBytePayload) {
  // Test with a small 2-byte payload like in OOTX frames
  const std::vector<std::uint8_t> data = {0xAB, 0xCD};
  const auto crc = calculateCRC32(data);

  EXPECT_EQ(crc, 0xE9FFC9D0);
}

TEST_F(CRC32Test, SingleBitDifference) {
  // Verify that a single bit difference changes the CRC
  const std::vector<std::uint8_t> data1 = {0x00, 0x00, 0x00};
  const std::vector<std::uint8_t> data2 = {0x01, 0x00,
                                           0x00}; // One bit different

  const auto crc1 = calculateCRC32(data1);
  const auto crc2 = calculateCRC32(data2);

  EXPECT_NE(crc1, crc2);
}

} // namespace lighthouse_protocol_decoder
