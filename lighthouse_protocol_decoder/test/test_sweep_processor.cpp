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

#include "lighthouse_protocol_decoder/sweep_processor.hpp"
#include "test_helpers.hpp"

#include <vector>

namespace lighthouse_protocol_decoder {

using test_helpers::createDataFrameContents;

// Alias for backward compatibility with existing test code
inline DataFrameContents createTestFrame(std::uint8_t sid, std::uint8_t npoly,
                                         std::uint32_t timestamp,
                                         std::uint32_t sync_offset = 0) {
  return createDataFrameContents(sid, npoly, timestamp, sync_offset);
}

class SweepProcessorTest : public ::testing::Test {
protected:
  void SetUp() override {
    sweeps_.clear();

    processor_ = std::make_unique<SweepProcessor>(
        [this](const SweepBlockRawData &sweep) { sweeps_.push_back(sweep); },
        nullptr);
  }

  std::unique_ptr<SweepProcessor> processor_;
  std::vector<SweepBlockRawData> sweeps_;
};

TEST_F(SweepProcessorTest, ConstructorWorksWithoutLogger) {
  std::vector<SweepBlockRawData> sweeps;
  auto processor_no_logger = std::make_unique<SweepProcessor>(
      [&sweeps](const SweepBlockRawData &sweep) { sweeps.push_back(sweep); });
  ASSERT_EQ(sweeps.size(), 0);
}

TEST_F(SweepProcessorTest, NoCallbackWithIncompleteData) {
  // Send frames from only 3 sensors - should not trigger callback
  // Valid npoly has bit 6 set (0x40), base station 0 is npoly & 0x03 = 0
  processor_->processFrame(createTestFrame(0, 0x40, 1000, 50000));
  processor_->processFrame(createTestFrame(1, 0x40, 1001));
  processor_->processFrame(createTestFrame(2, 0x40, 1002));

  ASSERT_EQ(sweeps_.size(), 0);
}

TEST_F(SweepProcessorTest, CompleteSweepTriggersCallback) {
  // Send frames from all 4 sensors
  // Valid npoly: bit 5 clear (0x00), base station 1 (npoly=0x00: (0/2)+1=1)
  // 3 sensors have valid npoly, 1 has sync_offset
  processor_->processFrame(createTestFrame(0, 0x00, 1000, 50000));
  processor_->processFrame(createTestFrame(1, 0x00, 1001));
  processor_->processFrame(createTestFrame(2, 0x00, 1002));
  processor_->processFrame(createTestFrame(3, 0x20, 1003)); // Invalid npoly

  ASSERT_EQ(sweeps_.size(), 1);
  EXPECT_EQ(sweeps_[0].base_station_id, 1);
  EXPECT_EQ(sweeps_[0].timestamp, 1000);
}

TEST_F(SweepProcessorTest, InvalidSweepWithZeroValidNpolys) {
  // All sensors have invalid npoly (bit 5 set = 0x20)
  processor_->processFrame(createTestFrame(0, 0x20, 1000, 50000));
  processor_->processFrame(createTestFrame(1, 0x20, 1001));
  processor_->processFrame(createTestFrame(2, 0x20, 1002));
  processor_->processFrame(createTestFrame(3, 0x20, 1003));

  ASSERT_EQ(sweeps_.size(), 0);
}

TEST_F(SweepProcessorTest, InvalidSweepWithTwoValidNpolys) {
  // Only 2 sensors have valid npoly (need exactly 3)
  processor_->processFrame(createTestFrame(0, 0x00, 1000, 50000));
  processor_->processFrame(createTestFrame(1, 0x00, 1001));
  processor_->processFrame(createTestFrame(2, 0x20, 1002)); // Invalid
  processor_->processFrame(createTestFrame(3, 0x20, 1003)); // Invalid

  ASSERT_EQ(sweeps_.size(), 0);
}

TEST_F(SweepProcessorTest, InvalidSweepWithFourValidNpolys) {
  // All 4 sensors have valid npoly (need exactly 3)
  processor_->processFrame(createTestFrame(0, 0x00, 1000, 50000));
  processor_->processFrame(createTestFrame(1, 0x00, 1001));
  processor_->processFrame(createTestFrame(2, 0x00, 1002));
  processor_->processFrame(createTestFrame(3, 0x00, 1003));

  ASSERT_EQ(sweeps_.size(), 0);
}

TEST_F(SweepProcessorTest, InvalidSweepWithNoSyncOffset) {
  // 3 valid npolys but no sensor has sync_offset
  processor_->processFrame(createTestFrame(0, 0x00, 1000));
  processor_->processFrame(createTestFrame(1, 0x00, 1001));
  processor_->processFrame(createTestFrame(2, 0x00, 1002));
  processor_->processFrame(createTestFrame(3, 0x20, 1003)); // Invalid npoly

  ASSERT_EQ(sweeps_.size(), 0);
}

TEST_F(SweepProcessorTest, InvalidSweepWithMultipleSyncOffsets) {
  // 3 valid npolys but 2 sensors have sync_offset (need exactly 1)
  processor_->processFrame(createTestFrame(0, 0x00, 1000, 50000));
  processor_->processFrame(createTestFrame(1, 0x00, 1001, 51000));
  processor_->processFrame(createTestFrame(2, 0x00, 1002));
  processor_->processFrame(createTestFrame(3, 0x20, 1003)); // Invalid npoly

  ASSERT_EQ(sweeps_.size(), 0);
}

TEST_F(SweepProcessorTest, InvalidSweepWithMismatchedBaseStations) {
  // Valid npoly values but different base stations
  // Base station 1: npoly = 0x00 (0b00000000) -> (0/2)+1=1
  // Base station 2: npoly = 0x02 (0b00000010) -> (2/2)+1=2
  processor_->processFrame(createTestFrame(0, 0x00, 1000, 50000));
  processor_->processFrame(createTestFrame(1, 0x02, 1001));
  processor_->processFrame(createTestFrame(2, 0x00, 1002));
  processor_->processFrame(createTestFrame(3, 0x20, 1003)); // Invalid npoly

  ASSERT_EQ(sweeps_.size(), 0);
}

TEST_F(SweepProcessorTest, NormalizedOffsetsCalculatedCorrectly) {
  // Send complete valid sweep
  processor_->processFrame(createTestFrame(0, 0x00, 1000, 50000));
  processor_->processFrame(createTestFrame(1, 0x00, 1005));
  processor_->processFrame(createTestFrame(2, 0x00, 1010));
  processor_->processFrame(createTestFrame(3, 0x20, 1015)); // Invalid npoly

  ASSERT_EQ(sweeps_.size(), 1);

  // Sensor 0 has the reference offset
  EXPECT_EQ(sweeps_[0].sensors[0].normalized_offset, 50000);

  // Other sensors should have calculated offsets based on timestamp deltas
  // Sensor 1: offset = 50000 + (1005 - 1000) = 50005
  EXPECT_EQ(sweeps_[0].sensors[1].normalized_offset, 50005);

  // Sensor 2: offset = 50000 + (1010 - 1000) = 50010
  EXPECT_EQ(sweeps_[0].sensors[2].normalized_offset, 50010);

  // Sensor 3: offset = 50000 + (1015 - 1000) = 50015
  EXPECT_EQ(sweeps_[0].sensors[3].normalized_offset, 50015);
}

TEST_F(SweepProcessorTest, OldFramesAreDiscarded) {
  // Send a frame from sensor 0
  processor_->processFrame(createTestFrame(0, 0x00, 1000, 50000));

  // Send a frame from sensor 1 that is too far in the future
  // kMaxTimestampDiffForSweep is 0x10000 (65536)
  processor_->processFrame(createTestFrame(1, 0x00, 0x11000));

  // Send frames to complete the sweep
  processor_->processFrame(createTestFrame(2, 0x00, 0x11001));
  processor_->processFrame(createTestFrame(3, 0x20, 0x11002)); // Invalid npoly

  // Should not trigger callback because sensor 0 was discarded
  ASSERT_EQ(sweeps_.size(), 0);
}

TEST_F(SweepProcessorTest, ResetClearsState) {
  // Send incomplete sweep
  processor_->processFrame(createTestFrame(0, 0x00, 1000, 50000));
  processor_->processFrame(createTestFrame(1, 0x00, 1001));

  // Reset
  processor_->reset();

  // Send complete sweep - should work
  processor_->processFrame(createTestFrame(0, 0x00, 2000, 60000));
  processor_->processFrame(createTestFrame(1, 0x00, 2001));
  processor_->processFrame(createTestFrame(2, 0x00, 2002));
  processor_->processFrame(createTestFrame(3, 0x20, 2003)); // Invalid npoly

  ASSERT_EQ(sweeps_.size(), 1);
}

TEST_F(SweepProcessorTest, MultipleConsecutiveSweeps) {
  // First sweep - base station 1 (npoly=0x00: (0/2)+1=1)
  processor_->processFrame(createTestFrame(0, 0x00, 1000, 50000));
  processor_->processFrame(createTestFrame(1, 0x00, 1001));
  processor_->processFrame(createTestFrame(2, 0x00, 1002));
  processor_->processFrame(createTestFrame(3, 0x20, 1003)); // Invalid npoly

  // Second sweep - base station 2 (npoly=0x02: (2/2)+1=2)
  processor_->processFrame(createTestFrame(0, 0x02, 2000, 60000));
  processor_->processFrame(createTestFrame(1, 0x02, 2001));
  processor_->processFrame(createTestFrame(2, 0x02, 2002));
  processor_->processFrame(createTestFrame(3, 0x20, 2003)); // Invalid npoly

  ASSERT_EQ(sweeps_.size(), 2);
  EXPECT_EQ(sweeps_[0].base_station_id, 1);
  EXPECT_EQ(sweeps_[1].base_station_id, 2);
}

TEST_F(SweepProcessorTest, DifferentSensorWithSyncOffset) {
  // Sensor 2 has the sync_offset instead of sensor 0
  processor_->processFrame(createTestFrame(0, 0x00, 1000));
  processor_->processFrame(createTestFrame(1, 0x00, 1005));
  processor_->processFrame(createTestFrame(2, 0x00, 1010, 50000));
  processor_->processFrame(createTestFrame(3, 0x20, 1015)); // Invalid npoly

  ASSERT_EQ(sweeps_.size(), 1);

  // Sensor 2 has the reference offset
  EXPECT_EQ(sweeps_[0].sensors[2].normalized_offset, 50000);

  // Timestamp should be the minimum from all sensors (matching Python)
  EXPECT_EQ(sweeps_[0].timestamp, 1000);

  // Other sensors calculated relative to sensor 2
  // Sensor 0: offset = 50000 + (1000 - 1010) = 49990 (with 24-bit wrap)
  EXPECT_EQ(sweeps_[0].sensors[0].normalized_offset,
            (50000 + timestampDiff(1000, 1010)) & 0xFFFFFF);
}

TEST_F(SweepProcessorTest, BaseStationExtractionFromNpoly) {
  // Test base station 3 (npoly=0x04: (4/2)+1=3)
  processor_->processFrame(
      createTestFrame(0, 0x04, 1000, 50000)); // 0x04 = 0b00000100
  processor_->processFrame(createTestFrame(1, 0x04, 1001));
  processor_->processFrame(createTestFrame(2, 0x04, 1002));
  processor_->processFrame(createTestFrame(3, 0x20, 1003)); // Invalid npoly

  ASSERT_EQ(sweeps_.size(), 1);
  EXPECT_EQ(sweeps_[0].base_station_id, 3);
}

TEST_F(SweepProcessorTest, BufferClearsAfterValidSweep) {
  // First sweep
  processor_->processFrame(createTestFrame(0, 0x00, 1000, 50000));
  processor_->processFrame(createTestFrame(1, 0x00, 1001));
  processor_->processFrame(createTestFrame(2, 0x00, 1002));
  processor_->processFrame(createTestFrame(3, 0x20, 1003)); // Invalid npoly

  ASSERT_EQ(sweeps_.size(), 1);

  // Send partial data for next sweep
  processor_->processFrame(createTestFrame(0, 0x02, 2000, 60000));
  processor_->processFrame(createTestFrame(1, 0x02, 2001));

  // Should not trigger another callback
  ASSERT_EQ(sweeps_.size(), 1);
}

} // namespace lighthouse_protocol_decoder
