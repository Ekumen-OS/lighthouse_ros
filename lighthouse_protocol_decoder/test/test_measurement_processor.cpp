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

#include <cmath>
#include <vector>

#include "lighthouse_protocol_decoder/measurement_processor.hpp"
#include "test_helpers.hpp"

namespace lighthouse_protocol_decoder
{

using test_helpers::createSweepBlockRawData;

// Alias for backward compatibility with existing test code
inline SweepBlockRawData
createTestSweepBlock(
  std::uint8_t base_station_id, std::uint32_t timestamp,
  std::uint32_t offset0, std::uint32_t offset1,
  std::uint32_t offset2, std::uint32_t offset3)
{
  return createSweepBlockRawData(
    base_station_id, timestamp, offset0, offset1,
    offset2, offset3);
}

// Test logger that prints to stdout for debugging
class TestLogger : public LoggerInterface
{
public:
  void debug(const std::string & message) override
  {
    std::cout << "[DEBUG] " << message << std::endl;
  }
  void info(const std::string & message) override
  {
    std::cout << "[INFO] " << message << std::endl;
  }
  void warning(const std::string & message) override
  {
    std::cout << "[WARN] " << message << std::endl;
  }
  void error(const std::string & message) override
  {
    std::cout << "[ERROR] " << message << std::endl;
  }
};

class MeasurementProcessorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    bearings_.clear();

    processor_ = std::make_unique<MeasurementProcessor>(
      [this](const SweepBlockBearings & bearings) {
        bearings_.push_back(bearings);
      },
      std::make_shared<TestLogger>());
  }

  std::unique_ptr<MeasurementProcessor> processor_;
  std::vector<SweepBlockBearings> bearings_;
};

TEST_F(MeasurementProcessorTest, ConstructorWorksWithoutLogger) {
  std::vector<SweepBlockBearings> bearings;
  auto processor_no_logger = std::make_unique<MeasurementProcessor>(
    [&bearings](const SweepBlockBearings & b) {bearings.push_back(b);});
  ASSERT_EQ(bearings.size(), 0);
}

TEST_F(MeasurementProcessorTest, NoCallbackWithSingleBlock) {
  // Send only one block - should not trigger callback (need 2 blocks)
  processor_->processBlock(
    createTestSweepBlock(1, 1000, 50000, 50100, 50200, 50300));

  ASSERT_EQ(bearings_.size(), 0);
}

TEST_F(MeasurementProcessorTest, CallbackWithTwoMatchingBlocks) {
  // Send two blocks from the same base station that form a matched pair
  auto [block_first, block_second] = test_helpers::createRealisticSweepBlocks(1, 1000, 2000);

  processor_->processBlock(block_first);
  processor_->processBlock(block_second);

  ASSERT_EQ(bearings_.size(), 1);
  EXPECT_EQ(bearings_[0].base_station_id, 1);
  EXPECT_EQ(bearings_[0].hardware_timestamp, 2000);
}

TEST_F(MeasurementProcessorTest, NoCallbackWhenOffsetDecreases) {
  // Second block has lower offset than first - should not match
  processor_->processBlock(
    createTestSweepBlock(1, 1000, 100000, 100100, 100200, 100300));
  processor_->processBlock(
    createTestSweepBlock(1, 2000, 50000, 50100, 50200, 50300));

  ASSERT_EQ(bearings_.size(), 0);
}

TEST_F(MeasurementProcessorTest, NoCallbackWhenTimestampDiffTooLarge) {
  // Timestamp difference exceeds kMaxTimestampDiffForBlockMatch
  processor_->processBlock(
    createTestSweepBlock(1, 1000, 50000, 50100, 50200, 50300));
  processor_->processBlock(
    createTestSweepBlock(1, 300000, 100000, 100100, 100200, 100300));

  ASSERT_EQ(bearings_.size(), 0);
}

TEST_F(MeasurementProcessorTest, BufferKeepsOnlyLastTwoBlocks) {
  // Send 3 blocks - buffer should only keep last 2
  processor_->processBlock(
    createTestSweepBlock(1, 1000, 50000, 50100, 50200, 50300));
  processor_->processBlock(
    createTestSweepBlock(1, 2000, 100000, 100100, 100200, 100300));
  processor_->processBlock(
    createTestSweepBlock(1, 3000, 150000, 150100, 150200, 150300));

  // Should have 2 callbacks: (1,2) and (2,3)
  ASSERT_EQ(bearings_.size(), 2);
}

TEST_F(MeasurementProcessorTest, DifferentBaseStationsProcessedSeparately) {
  // Send blocks from two different base stations
  processor_->processBlock(
    createTestSweepBlock(1, 1000, 50000, 50100, 50200, 50300));
  processor_->processBlock(
    createTestSweepBlock(2, 1100, 50000, 50100, 50200, 50300));
  processor_->processBlock(
    createTestSweepBlock(1, 2000, 100000, 100100, 100200, 100300));
  processor_->processBlock(
    createTestSweepBlock(2, 2100, 100000, 100100, 100200, 100300));

  // Should have 2 callbacks, one for each base station
  ASSERT_EQ(bearings_.size(), 2);
  EXPECT_EQ(bearings_[0].base_station_id, 1);
  EXPECT_EQ(bearings_[1].base_station_id, 2);
}

TEST_F(MeasurementProcessorTest, ResetClearsState) {
  // Send one block
  processor_->processBlock(
    createTestSweepBlock(1, 1000, 50000, 50100, 50200, 50300));

  // Reset
  processor_->reset();

  // Send one block - should not trigger callback
  processor_->processBlock(
    createTestSweepBlock(1, 2000, 100000, 100100, 100200, 100300));

  ASSERT_EQ(bearings_.size(), 0);
}

TEST_F(MeasurementProcessorTest, BearingCalculationBasicValues) {
  // Test with specific values that should produce predictable results
  // Using base station 1 with period 959000.0 / 2.0 = 479500.0
  const std::uint32_t offset_first = 100000;
  const std::uint32_t offset_second = 200000;

  processor_->processBlock(
    createTestSweepBlock(
      1, 1000, offset_first, offset_first, offset_first, offset_first));
  processor_->processBlock(
    createTestSweepBlock(
      1, 2000, offset_second, offset_second, offset_second, offset_second));

  ASSERT_EQ(bearings_.size(), 1);

  // Verify all sensors have the same angles since they have the same offsets
  for (std::size_t i = 1; i < kPulseProcessorNSensors; ++i) {
    EXPECT_NEAR(
      bearings_[0].sensor_angles[i].azimuth,
      bearings_[0].sensor_angles[0].azimuth, 0.001);
    EXPECT_NEAR(
      bearings_[0].sensor_angles[i].elevation,
      bearings_[0].sensor_angles[0].elevation, 0.001);
  }
}

TEST_F(MeasurementProcessorTest, BearingCalculationDifferentSensors) {
  // Test with different offset values for each sensor
  processor_->processBlock(
    createTestSweepBlock(1, 1000, 50000, 60000, 70000, 80000));
  processor_->processBlock(
    createTestSweepBlock(1, 2000, 100000, 110000, 120000, 130000));

  ASSERT_EQ(bearings_.size(), 1);

  // Each sensor should have different angles
  for (std::size_t i = 1; i < kPulseProcessorNSensors; ++i) {
    // Azimuth should be different
    EXPECT_NE(
      bearings_[0].sensor_angles[i].azimuth,
      bearings_[0].sensor_angles[0].azimuth);
  }
}

TEST_F(MeasurementProcessorTest, BearingValuesInReasonableRange) {
  // Verify that calculated bearings are within reasonable ranges
  processor_->processBlock(
    createTestSweepBlock(1, 1000, 50000, 60000, 70000, 80000));
  processor_->processBlock(
    createTestSweepBlock(1, 2000, 100000, 110000, 120000, 130000));

  ASSERT_EQ(bearings_.size(), 1);

  // Check all sensor angles are within reasonable bounds
  for (std::size_t i = 0; i < kPulseProcessorNSensors; ++i) {
    // Azimuth should be in [-180, 180] degrees
    EXPECT_GE(bearings_[0].sensor_angles[i].azimuth, -180.0);
    EXPECT_LE(bearings_[0].sensor_angles[i].azimuth, 180.0);

    // Elevation should be in reasonable range (approximate)
    EXPECT_GE(bearings_[0].sensor_angles[i].elevation, -90.0);
    EXPECT_LE(bearings_[0].sensor_angles[i].elevation, 90.0);
  }
}

TEST_F(MeasurementProcessorTest, MultipleConsecutiveMeasurements) {
  // Send multiple sweep pairs in sequence
  for (int i = 0; i < 5; ++i) {
    processor_->processBlock(
      createTestSweepBlock(
        1, 1000 + i * 1000, 50000 + i * 10000, 50100 + i * 10000,
        50200 + i * 10000, 50300 + i * 10000));
  }

  // Should get 4 callbacks (pairs: 0-1, 1-2, 2-3, 3-4)
  ASSERT_EQ(bearings_.size(), 4);

  // All should be from base station 1
  for (const auto & bearing : bearings_) {
    EXPECT_EQ(bearing.base_station_id, 1);
  }
}

TEST_F(MeasurementProcessorTest, TimestampAssignment) {
  // Verify that hardware_timestamp is set to the current block's timestamp
  processor_->processBlock(
    createTestSweepBlock(1, 1000, 50000, 50100, 50200, 50300));
  processor_->processBlock(
    createTestSweepBlock(1, 2000, 100000, 100100, 100200, 100300));

  ASSERT_EQ(bearings_.size(), 1);
  EXPECT_EQ(bearings_[0].hardware_timestamp, 2000);
}

TEST_F(MeasurementProcessorTest, BaseStationPeriodUsedCorrectly) {
  // Test with different base stations - they should use different periods
  // and produce different results even with the same offsets

  // Base station 1
  processor_->processBlock(
    createTestSweepBlock(1, 1000, 50000, 50000, 50000, 50000));
  processor_->processBlock(
    createTestSweepBlock(1, 2000, 100000, 100000, 100000, 100000));

  ASSERT_EQ(bearings_.size(), 1);
  const double azimuth_bs1 = bearings_[0].sensor_angles[0].azimuth;

  bearings_.clear();
  processor_->reset();

  // Base station 2 (different period: 957000.0 / 2.0 = 478500.0)
  processor_->processBlock(
    createTestSweepBlock(2, 1000, 50000, 50000, 50000, 50000));
  processor_->processBlock(
    createTestSweepBlock(2, 2000, 100000, 100000, 100000, 100000));

  ASSERT_EQ(bearings_.size(), 1);
  const double azimuth_bs2 = bearings_[0].sensor_angles[0].azimuth;

  // Azimuth values should be different due to different periods
  EXPECT_NE(azimuth_bs1, azimuth_bs2);
}

TEST_F(MeasurementProcessorTest, EdgeCaseZeroOffsets) {
  // Test with zero offsets (edge case)
  processor_->processBlock(createTestSweepBlock(1, 1000, 0, 0, 0, 0));
  processor_->processBlock(
    createTestSweepBlock(1, 2000, 100000, 100000, 100000, 100000));

  ASSERT_EQ(bearings_.size(), 1);
  // Should not crash and should produce valid results
  EXPECT_FALSE(std::isnan(bearings_[0].sensor_angles[0].azimuth));
  EXPECT_FALSE(std::isnan(bearings_[0].sensor_angles[0].elevation));
}

TEST_F(MeasurementProcessorTest, NonMatchingPairSkipped) {
  // Send two blocks where second one doesn't match
  processor_->processBlock(
    createTestSweepBlock(1, 1000, 100000, 100100, 100200, 100300));
  processor_->processBlock(
    createTestSweepBlock(
      1, 2000, 50000, 50100, 50200,
      50300));                                           // Offset decreased

  ASSERT_EQ(bearings_.size(), 0);

  // Send a third block that matches the second
  processor_->processBlock(
    createTestSweepBlock(1, 3000, 80000, 80100, 80200, 80300));

  // Should get callback for pair (2, 3)
  ASSERT_EQ(bearings_.size(), 1);
  EXPECT_EQ(bearings_[0].hardware_timestamp, 3000);
}

TEST_F(MeasurementProcessorTest, SpecificBearingAngles10DegMinus30Deg) {
  // Test case with specific target bearings: azimuth = 10°, elevation = 30°
  // Offsets calculated using the corrected formula: atan(sin(beta/2) /
  // tan(p/2))

  const std::uint32_t offset_0_first = 147218;  // phase0 = 1.929 rad
  const std::uint32_t offset_1_first = 358920;  // phase1 = 4.703 rad

  processor_->processBlock(
    createTestSweepBlock(
      1, 1000, offset_0_first, offset_0_first, offset_0_first, offset_0_first));
  processor_->processBlock(
    createTestSweepBlock(
      1, 2000, offset_1_first, offset_1_first, offset_1_first, offset_1_first));

  ASSERT_EQ(bearings_.size(), 1);

  // Check azimuth is approximately 10 degrees
  EXPECT_NEAR(bearings_[0].sensor_angles[0].azimuth, 10.0, 1.0);

  // Check elevation is approximately 30 degrees
  EXPECT_NEAR(bearings_[0].sensor_angles[0].elevation, 30.0, 1.0);

  // All sensors should have the same angles since they have the same offsets
  for (std::size_t i = 1; i < kPulseProcessorNSensors; ++i) {
    EXPECT_NEAR(
      bearings_[0].sensor_angles[i].azimuth,
      bearings_[0].sensor_angles[0].azimuth, 0.001);
    EXPECT_NEAR(
      bearings_[0].sensor_angles[i].elevation,
      bearings_[0].sensor_angles[0].elevation, 0.001);
  }
}

TEST_F(MeasurementProcessorTest, ValidationPassesWithRealisticGeometry) {
  // Default geometry should pass all validation checks
  auto [block_first, block_second] = test_helpers::createRealisticSweepBlocks(1, 1000, 2000);

  processor_->processBlock(block_first);
  processor_->processBlock(block_second);

  ASSERT_EQ(bearings_.size(), 1);
  EXPECT_EQ(bearings_[0].base_station_id, 1);
  EXPECT_EQ(bearings_[0].hardware_timestamp, 2000);
}

TEST_F(MeasurementProcessorTest, ValidationRejectsExcessiveAzimuthSpread) {
  // Place deck very close (0.3m) and offset to the side to create large azimuth spread
  // At 0.3m distance, 33.5mm baseline creates ~6.4° spread >> 1.28° limit
  auto [block_first, block_second] = test_helpers::createSweepBlocksFromGeometry(
    0.0, 0.0, 0.0,      // station at origin
    0.3, 0.0, 0.0,      // deck 30cm in front (same height)
    1, 1000, 2000);

  processor_->processBlock(block_first);
  processor_->processBlock(block_second);

  // Should be rejected due to excessive azimuth spread
  ASSERT_EQ(bearings_.size(), 0);
}

TEST_F(MeasurementProcessorTest, ValidationRejectsExcessiveElevationSpread) {
  // Place deck very close (0.3m) and offset vertically to create large elevation spread
  // At 0.3m distance, 33.5mm baseline creates ~6.4° spread >> 1.28° limit
  auto [block_first, block_second] = test_helpers::createSweepBlocksFromGeometry(
    0.0, 0.0, 0.0,      // station at origin
    0.0, 0.0, -0.3,     // deck 30cm below (no forward offset)
    1, 1000, 2000);

  processor_->processBlock(block_first);
  processor_->processBlock(block_second);

  // Should be rejected due to excessive elevation spread
  ASSERT_EQ(bearings_.size(), 0);
}

TEST_F(MeasurementProcessorTest, ValidationRejectsCounterclockwiseWinding) {
  // Place deck 2m in front but above the station (not below)
  // This creates counter-clockwise winding order
  auto [block_first, block_second] = test_helpers::createSweepBlocksFromGeometry(
    0.0, 0.0, 0.0,      // station at origin
    2.0, 0.0, +0.02,    // deck 2m in front, 2cm ABOVE (not below)
    1, 1000, 2000);

  processor_->processBlock(block_first);
  processor_->processBlock(block_second);

  // Should be rejected due to wrong winding order
  ASSERT_EQ(bearings_.size(), 0);
}

TEST_F(MeasurementProcessorTest, ValidationAcceptsBoundaryAzimuthSpread) {
  // Place deck at distance where angular spread is just under the limit
  // At 1.6m: spread = atan(0.0335/1.6) = 1.20° < 1.28° limit
  auto [block_first, block_second] = test_helpers::createSweepBlocksFromGeometry(
    0.0, 0.0, 0.0,      // station at origin
    1.6, 0.0, -0.02,    // deck 1.6m in front, 2cm below
    1, 1000, 2000);

  processor_->processBlock(block_first);
  processor_->processBlock(block_second);

  // Should pass validation (spread ~1.20° < 1.28°)
  ASSERT_EQ(bearings_.size(), 1);
}

TEST_F(MeasurementProcessorTest, ValidationRejectsBoundaryExceedingAzimuthSpread) {
  // Place deck at distance where angular spread exceeds the limit
  // At 0.6m with Y-baseline of 15mm: spread = atan(0.015/0.6) = 1.43° > 1.28° limit
  auto [block_first, block_second] = test_helpers::createSweepBlocksFromGeometry(
    0.0, 0.0, 0.0,      // station at origin
    0.6, 0.0, -0.02,    // deck 0.6m in front, 2cm below
    1, 1000, 2000);

  processor_->processBlock(block_first);
  processor_->processBlock(block_second);

  // Should be rejected (azimuth spread ~1.43° > 1.28°)
  ASSERT_EQ(bearings_.size(), 0);
}

}    // namespace lighthouse_protocol_decoder
