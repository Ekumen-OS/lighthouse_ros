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
using test_helpers::StdoutLogger;

// Helper function to convert desired bearing angles to timing offsets
// This inverts the FULL 4-step decodification process
std::pair<std::uint32_t, std::uint32_t>
anglesToOffsets(double azimuth_deg, double elevation_deg)
{
  // Use channel 1 period as reference
  constexpr std::uint32_t kPeriod = 479500;

  // Convert to radians
  const double azimuth = azimuth_deg * M_PI / 180.0;
  const double elevation = elevation_deg * M_PI / 180.0;

  // Reverse Step 4: Convert spherical coordinates to V1 angles (angleH, angleV)
  // Derived from the forward transformation equations:
  //   angleH has a direct relationship to azimuth
  //   angleV is derived from the plane intersection geometry
  const double angleH = azimuth;
  const double angleV = std::atan2(
    std::sin(elevation),
    std::cos(elevation) * std::cos(azimuth)
  );

  // Reverse Step 3: Convert V1 angles back to V2 angles
  const double tant = std::tan(M_PI / 6.0);
  const double d = std::asin(std::tan(angleV) * tant * std::cos(angleH));
  const double v2_angle_1 = angleH - d;
  const double v2_angle_2 = angleH + d;

  // Reverse Step 2: Convert V2 angles to phase angles
  const double phase_0 = v2_angle_1 + M_PI - M_PI / 3.0;
  const double phase_1 = v2_angle_2 + M_PI + M_PI / 3.0;

  // Reverse Step 1: Convert phase angles to timing offsets
  const auto offset_0 = static_cast<std::uint32_t>((phase_0 / (2.0 * M_PI)) * kPeriod);
  const auto offset_1 = static_cast<std::uint32_t>((phase_1 / (2.0 * M_PI)) * kPeriod);

  return {offset_0, offset_1};
}

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
      std::make_shared<StdoutLogger>());
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
  auto [block1a, block1b] = test_helpers::createRealisticSweepBlocks(1, 1000, 2000);
  auto [block2a, block2b] = test_helpers::createRealisticSweepBlocks(1, 3000, 4000);
  auto [block3a, block3b] = test_helpers::createRealisticSweepBlocks(1, 5000, 6000);

  processor_->processBlock(block1a);
  processor_->processBlock(block1b);
  processor_->processBlock(block2a);
  processor_->processBlock(block2b);
  processor_->processBlock(block3a);
  processor_->processBlock(block3b);

  // Should have 3 callbacks: (1a,1b), (2a,2b), (3a,3b)
  ASSERT_EQ(bearings_.size(), 3);
}

TEST_F(MeasurementProcessorTest, DifferentBaseStationsProcessedSeparately) {
  // Send blocks from two different base stations
  auto [block1a, block1b] = test_helpers::createRealisticSweepBlocks(1, 1000, 2000);
  auto [block2a, block2b] = test_helpers::createRealisticSweepBlocks(2, 1100, 2100);

  processor_->processBlock(block1a);
  processor_->processBlock(block2a);
  processor_->processBlock(block1b);
  processor_->processBlock(block2b);

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
  // Test with realistic geometry - sensors have slightly different angles
  auto [block_first, block_second] = test_helpers::createRealisticSweepBlocks(1, 1000, 2000);

  processor_->processBlock(block_first);
  processor_->processBlock(block_second);

  ASSERT_EQ(bearings_.size(), 1);

  // With realistic geometry, sensors have slightly different angles
  // Check that at least some sensors have measurable angle differences
  bool found_difference = false;
  for (std::size_t i = 1; i < kPulseProcessorNSensors; ++i) {
    if (std::abs(
        bearings_[0].sensor_angles[i].azimuth -
        bearings_[0].sensor_angles[0].azimuth) > 0.001)
    {
      found_difference = true;
      break;
    }
  }
  EXPECT_TRUE(found_difference) << "Sensors should have slightly different angles";
}

TEST_F(MeasurementProcessorTest, BearingValuesInReasonableRange) {
  // Verify that calculated bearings are within reasonable ranges
  auto [block_first, block_second] = test_helpers::createRealisticSweepBlocks(1, 1000, 2000);

  processor_->processBlock(block_first);
  processor_->processBlock(block_second);

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
  for (int i = 0; i < 4; ++i) {
    auto [block_first, block_second] = test_helpers::createRealisticSweepBlocks(
      1, 1000 + i * 10000, 2000 + i * 10000);
    processor_->processBlock(block_first);
    processor_->processBlock(block_second);
  }

  // Should get 4 callbacks (one per pair)
  ASSERT_EQ(bearings_.size(), 4);

  // All should be from base station 1
  for (const auto & bearing : bearings_) {
    EXPECT_EQ(bearing.base_station_id, 1);
  }
}

TEST_F(MeasurementProcessorTest, TimestampAssignment) {
  // Verify that hardware_timestamp is set to the current block's timestamp
  auto [block_first, block_second] = test_helpers::createRealisticSweepBlocks(1, 1000, 2000);

  processor_->processBlock(block_first);
  processor_->processBlock(block_second);

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

// ============================================================================
// Angle Calculation Formula Validation Tests (Parameterized)
// ============================================================================
// These tests verify the correct FULL 4-step V2 decodification process:
//   Step 1: Timing offsets → phase angles
//   Step 2: Phase angles → V2 angles (with ±60° phase corrections)
//   Step 3: V2 angles → V1 angles (angleH, angleV plane intersection)
//   Step 4: V1 angles → ray → spherical coordinates (azimuth, elevation)
//
// Reference: Crazyflie firmware pulse_processor_v2.c and lighthouse_geometry.c
//
// These tests verify the final output is true spherical azimuth/elevation,
// not the intermediate angleH/angleV values from Step 3.

struct AngleTestCase
{
  double azimuth_deg;    // Final spherical azimuth
  double elevation_deg;  // Final spherical elevation
  double tolerance_deg;
  std::string description;
};

class AngleFormulaTest : public MeasurementProcessorTest,
  public ::testing::WithParamInterface<AngleTestCase>
{
};

TEST_P(AngleFormulaTest, CorrectAngleCalculation) {
  const auto & test_case = GetParam();

  // Convert desired spherical coordinates to offsets using the inverse of the full 4-step process
  const auto [offset_0, offset_1] = anglesToOffsets(test_case.azimuth_deg, test_case.elevation_deg);

  // Send two blocks with calculated offsets
  processor_->processBlock(
    createTestSweepBlock(1, 1000, offset_0, offset_0, offset_0, offset_0));
  processor_->processBlock(
    createTestSweepBlock(1, 2000, offset_1, offset_1, offset_1, offset_1));

  ASSERT_EQ(bearings_.size(), 1) << "Failed for: " << test_case.description;

  // Check azimuth (final spherical coordinate)
  EXPECT_NEAR(
    bearings_[0].sensor_angles[0].azimuth, test_case.azimuth_deg,
    test_case.tolerance_deg)
    << "Azimuth mismatch for: " << test_case.description;

  // Check elevation (final spherical coordinate)
  EXPECT_NEAR(
    bearings_[0].sensor_angles[0].elevation, test_case.elevation_deg,
    test_case.tolerance_deg)
    << "Elevation mismatch for: " << test_case.description;

  // All sensors should have the same angles since they have the same offsets
  for (std::size_t i = 1; i < kPulseProcessorNSensors; ++i) {
    EXPECT_NEAR(
      bearings_[0].sensor_angles[i].azimuth,
      bearings_[0].sensor_angles[0].azimuth, 0.001)
      << "Sensor " << i << " azimuth differs for: " << test_case.description;
    EXPECT_NEAR(
      bearings_[0].sensor_angles[i].elevation,
      bearings_[0].sensor_angles[0].elevation, 0.001)
      << "Sensor " << i << " elevation differs for: " << test_case.description;
  }
}

INSTANTIATE_TEST_SUITE_P(
  AngleCalculationTests,
  AngleFormulaTest,
  ::testing::Values(
    // Basic center point
    AngleTestCase{0.0, 0.0, 0.5, "Zero bearing (origin)"},

    // Pure azimuth variations (angleH, angleV=0)
    AngleTestCase{30.0, 0.0, 0.5, "Positive azimuth, zero elevation"},
    AngleTestCase{-30.0, 0.0, 0.5, "Negative azimuth, zero elevation"},
    AngleTestCase{45.0, 0.0, 0.5, "Large positive azimuth"},
    AngleTestCase{-45.0, 0.0, 0.5, "Large negative azimuth"},

    // Pure elevation variations (angleH=0, angleV) - critical tests!
    AngleTestCase{0.0, 30.0, 0.5, "Zero azimuth, positive elevation"},
    AngleTestCase{0.0, -30.0, 0.5, "Zero azimuth, negative elevation"},
    AngleTestCase{0.0, 50.0, 0.5, "Large positive elevation"},
    AngleTestCase{0.0, -50.0, 0.5, "Large negative elevation"},

    // Combined angle variations
    AngleTestCase{10.0, 30.0, 0.5, "Both angles positive (original test case)"},
    AngleTestCase{20.0, 20.0, 0.5, "Both angles positive equal"},
    AngleTestCase{-20.0, -20.0, 0.5, "Both angles negative"},
    AngleTestCase{20.0, -20.0, 0.5, "Positive azimuth, negative elevation"},
    AngleTestCase{-20.0, 20.0, 0.5, "Negative azimuth, positive elevation"},

    // Extreme angles
    AngleTestCase{45.0, 40.0, 0.5, "Extreme angles combination"},
    AngleTestCase{-60.0, 30.0, 0.5, "Large negative azimuth, positive elevation"},
    AngleTestCase{60.0, -30.0, 0.5, "Large positive azimuth, negative elevation"},

    // Symmetry verification
    AngleTestCase{0.0, 25.0, 0.5, "Symmetry test: +25° elevation"},
    AngleTestCase{0.0, -25.0, 0.5, "Symmetry test: -25° elevation"},
    AngleTestCase{15.0, 0.0, 0.5, "Symmetry test: +15° azimuth"},
    AngleTestCase{-15.0, 0.0, 0.5, "Symmetry test: -15° azimuth"}
  )
);

// Test with specific angle values to verify the full 4-step decodification
TEST_F(MeasurementProcessorTest, SpecificBearingAngles10Deg30Deg) {
  // Test case with specific target spherical coordinates: azimuth = 10°, elevation = 30°
  const double target_azimuth = 10.0;
  const double target_elevation = 30.0;

  // Generate offsets using the inverse of the full 4-step process
  const auto [offset_0, offset_1] = anglesToOffsets(target_azimuth, target_elevation);

  processor_->processBlock(
    createTestSweepBlock(1, 1000, offset_0, offset_0, offset_0, offset_0));
  processor_->processBlock(
    createTestSweepBlock(1, 2000, offset_1, offset_1, offset_1, offset_1));

  ASSERT_EQ(bearings_.size(), 1);

  // Check azimuth matches target
  EXPECT_NEAR(bearings_[0].sensor_angles[0].azimuth, target_azimuth, 0.5);

  // Check elevation matches target
  EXPECT_NEAR(bearings_[0].sensor_angles[0].elevation, target_elevation, 0.5);

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

}    // namespace lighthouse_protocol_decoder
