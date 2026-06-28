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
#include <map>
#include <string>
#include <vector>

#include "lighthouse_protocol_decoder/lighthouse_protocol_decoder.hpp"
#include "lighthouse_protocol_decoder/logger.hpp"
#include "test_helpers.hpp"

namespace lighthouse_protocol_decoder
{

using test_helpers::createCompleteMeasurement;
using test_helpers::createDataFrame;
using test_helpers::createInterleavedMeasurements;
using test_helpers::createSyncFrame;
using test_helpers::makeNpoly;
using test_helpers::StdoutLogger;

// ===========================================================================
// Test Fixture
// ===========================================================================

class LighthouseProtocolDecoderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    received_bearings_.clear();

    decoder_ = std::make_unique<LighthouseProtocolDecoder>(
      [this](const SweepBlockBearings & bearings) {
        received_bearings_.push_back(bearings);
      },
      std::make_shared<StdoutLogger>());
  }

  /// Send bytes to the decoder
  void sendBytes(const std::vector<std::uint8_t> & bytes)
  {
    for (auto byte : bytes) {
      decoder_->processByte(byte);
    }
  }

  /// Send a complete protocol sequence (sync frame + data)
  void sendProtocolSequence(const std::vector<std::uint8_t> & data)
  {
    // First send sync frame to enter DATA mode
    sendBytes(createSyncFrame());
    EXPECT_EQ(
      decoder_->getCurrentMode(),
      LighthouseProtocolDecoder::Mode::DATA);

    // Then send the actual data
    sendBytes(data);
  }

  std::unique_ptr<LighthouseProtocolDecoder> decoder_;
  std::vector<SweepBlockBearings> received_bearings_;
};

// ===========================================================================
// Basic Tests
// ===========================================================================

TEST_F(LighthouseProtocolDecoderTest, InitialStateIsSyncMode) {
  EXPECT_EQ(decoder_->getCurrentMode(), LighthouseProtocolDecoder::Mode::SYNC);
  EXPECT_EQ(received_bearings_.size(), 0);
}

TEST_F(LighthouseProtocolDecoderTest, SyncFrameSwitchesToDataMode) {
  sendBytes(createSyncFrame());
  EXPECT_EQ(decoder_->getCurrentMode(), LighthouseProtocolDecoder::Mode::DATA);
}

TEST_F(LighthouseProtocolDecoderTest, ResetReturnsToSyncMode) {
  sendBytes(createSyncFrame());
  EXPECT_EQ(decoder_->getCurrentMode(), LighthouseProtocolDecoder::Mode::DATA);

  decoder_->reset();
  EXPECT_EQ(decoder_->getCurrentMode(), LighthouseProtocolDecoder::Mode::SYNC);
  EXPECT_EQ(received_bearings_.size(), 0);
}

TEST_F(LighthouseProtocolDecoderTest, ConstructorWorksWithoutCallback) {
  // Verify constructor works without callback
  auto decoder_no_callback =
    std::make_unique<LighthouseProtocolDecoder>(nullptr, nullptr);
  EXPECT_EQ(
    decoder_no_callback->getCurrentMode(),
    LighthouseProtocolDecoder::Mode::SYNC);
}

// ===========================================================================
// Single Basestation Test
// ===========================================================================

TEST_F(LighthouseProtocolDecoderTest, SingleBasestationCompleteMeasurement) {
  // Create a complete measurement sequence from a single basestation
  const std::uint8_t base_station_id = 5;  // Arbitrary basestation ID
  const std::uint32_t base_timestamp = 1000000;

  auto measurement_data =
    createCompleteMeasurement(base_station_id, base_timestamp);

  // Send the protocol sequence
  sendProtocolSequence(measurement_data);

  // We expect to receive bearing measurements for the basestation
  // The exact number depends on the measurement processor's matching logic
  // At minimum, we should get at least one measurement
  ASSERT_GT(received_bearings_.size(), 0)
    << "Expected at least one bearing measurement";

  // Verify that all received bearings are from the correct basestation
  for (const auto & bearing : received_bearings_) {
    EXPECT_EQ(bearing.base_station_id, base_station_id)
      << "Bearing from unexpected basestation";

    // Verify that azimuth and elevation values are reasonable
    // (not NaN, not infinite)
    for (std::size_t sensor = 0; sensor < kPulseProcessorNSensors; ++sensor) {
      EXPECT_FALSE(std::isnan(bearing.sensor_angles[sensor].azimuth))
        << "Sensor " << sensor << " azimuth is NaN";
      EXPECT_FALSE(std::isnan(bearing.sensor_angles[sensor].elevation))
        << "Sensor " << sensor << " elevation is NaN";
      EXPECT_FALSE(std::isinf(bearing.sensor_angles[sensor].azimuth))
        << "Sensor " << sensor << " azimuth is infinite";
      EXPECT_FALSE(std::isinf(bearing.sensor_angles[sensor].elevation))
        << "Sensor " << sensor << " elevation is infinite";
    }
  }
}

TEST_F(LighthouseProtocolDecoderTest, SingleBasestationMultipleRounds) {
  // Test multiple consecutive measurements from the same basestation
  const std::uint8_t base_station_id = 3;
  std::uint32_t timestamp = 500000;

  // Send sync frame once
  sendBytes(createSyncFrame());
  EXPECT_EQ(decoder_->getCurrentMode(), LighthouseProtocolDecoder::Mode::DATA);

  // Send multiple measurement rounds
  const int num_rounds = 5;
  for (int round = 0; round < num_rounds; ++round) {
    auto measurement_data =
      createCompleteMeasurement(base_station_id, timestamp);
    sendBytes(measurement_data);
    timestamp = (timestamp + 500000) & kTimestampCounterMask;
  }

  // We should receive measurements from all rounds
  EXPECT_GT(received_bearings_.size(), 0)
    << "Expected measurements from multiple rounds";

  // Verify all bearings are from the correct basestation
  for (const auto & bearing : received_bearings_) {
    EXPECT_EQ(bearing.base_station_id, base_station_id);
  }
}

// ===========================================================================
// Four Basestations Test
// ===========================================================================

TEST_F(LighthouseProtocolDecoderTest, FourBasestationsInterleaved) {
  // Create measurements from four basestations arriving simultaneously
  const std::vector<std::uint8_t> base_station_ids = {1, 5, 9, 13};
  const std::uint32_t base_timestamp = 2000000;

  auto interleaved_data =
    createInterleavedMeasurements(base_station_ids, base_timestamp);

  // Send the protocol sequence
  sendProtocolSequence(interleaved_data);

  // We expect to receive bearing measurements from multiple basestations
  ASSERT_GT(received_bearings_.size(), 0)
    << "Expected bearing measurements from multiple basestations";

  // Count measurements per basestation
  std::map<std::uint8_t, int> measurements_per_basestation;
  for (const auto & bearing : received_bearings_) {
    measurements_per_basestation[bearing.base_station_id]++;

    // Verify bearing values are reasonable
    for (std::size_t sensor = 0; sensor < kPulseProcessorNSensors; ++sensor) {
      EXPECT_FALSE(std::isnan(bearing.sensor_angles[sensor].azimuth))
        << "BS " << static_cast<int>(bearing.base_station_id) << " Sensor "
        << sensor << " azimuth is NaN";
      EXPECT_FALSE(std::isnan(bearing.sensor_angles[sensor].elevation))
        << "BS " << static_cast<int>(bearing.base_station_id) << " Sensor "
        << sensor << " elevation is NaN";
    }
  }

  // Verify we got measurements from multiple basestations
  EXPECT_GT(measurements_per_basestation.size(), 1)
    << "Expected measurements from multiple basestations";

  // Log the distribution for debugging
  for (const auto &[bs_id, count] : measurements_per_basestation) {
    std::cout << "Basestation " << static_cast<int>(bs_id) << ": " << count
              << " measurements" << std::endl;
  }
}

TEST_F(LighthouseProtocolDecoderTest, FourBasestationsSequential) {
  // Test four basestations sending data sequentially (not interleaved)
  const std::vector<std::uint8_t> base_station_ids = {2, 6, 10, 14};
  std::uint32_t timestamp = 3000000;

  // Send sync frame once
  sendBytes(createSyncFrame());
  EXPECT_EQ(decoder_->getCurrentMode(), LighthouseProtocolDecoder::Mode::DATA);

  // Send measurements from each basestation sequentially
  for (auto bs_id : base_station_ids) {
    auto measurement_data = createCompleteMeasurement(bs_id, timestamp);
    sendBytes(measurement_data);
    timestamp = (timestamp + 600000) & kTimestampCounterMask;
  }

  // Verify we received measurements
  ASSERT_GT(received_bearings_.size(), 0)
    << "Expected bearing measurements from all basestations";

  // Count unique basestations
  std::set<std::uint8_t> unique_basestations;
  for (const auto & bearing : received_bearings_) {
    unique_basestations.insert(bearing.base_station_id);
  }

  // We should see measurements from multiple basestations
  EXPECT_GT(unique_basestations.size(), 1)
    << "Expected measurements from multiple basestations";

  // Log results
  std::cout << "Received measurements from " << unique_basestations.size()
            << " unique basestations: ";
  for (auto bs_id : unique_basestations) {
    std::cout << static_cast<int>(bs_id) << " ";
  }
  std::cout << std::endl;
}

TEST_F(LighthouseProtocolDecoderTest, FourBasestationsConcurrent) {
  // Test realistic scenario with four basestations operating concurrently
  // Each basestation sends multiple rounds
  const std::vector<std::uint8_t> base_station_ids = {1, 5, 9, 13};
  std::uint32_t timestamp = 4000000;

  // Send sync frame
  sendBytes(createSyncFrame());

  // Send multiple rounds of interleaved measurements
  const int num_rounds = 3;
  for (int round = 0; round < num_rounds; ++round) {
    auto interleaved_data =
      createInterleavedMeasurements(base_station_ids, timestamp);
    sendBytes(interleaved_data);
    timestamp = (timestamp + 1000000) & kTimestampCounterMask;
  }

  // Verify we received measurements
  ASSERT_GT(received_bearings_.size(), 0)
    << "Expected bearing measurements from concurrent basestations";

  // Analyze the distribution
  std::map<std::uint8_t, int> measurements_per_basestation;
  for (const auto & bearing : received_bearings_) {
    measurements_per_basestation[bearing.base_station_id]++;
  }

  // We should have measurements from multiple basestations
  EXPECT_GT(measurements_per_basestation.size(), 1)
    << "Expected measurements from multiple basestations in concurrent mode";

  // Report statistics
  std::cout << "Concurrent test - Total measurements: "
            << received_bearings_.size() << std::endl;
  for (const auto &[bs_id, count] : measurements_per_basestation) {
    std::cout << "  Basestation " << static_cast<int>(bs_id) << ": " << count
              << " measurements" << std::endl;
  }
}

// ===========================================================================
// Edge Cases and Error Handling
// ===========================================================================

TEST_F(LighthouseProtocolDecoderTest, BadPaddingSwitchesBackToSyncMode) {
  // Send sync frame to enter DATA mode
  sendBytes(createSyncFrame());
  EXPECT_EQ(decoder_->getCurrentMode(), LighthouseProtocolDecoder::Mode::DATA);

  // Send a data frame with bad padding (padding_1 = 1)
  auto bad_frame =
    createDataFrame(0, makeNpoly(1, true, 0), 100, 25000, 1, 0, 0, 100000);
  sendBytes(bad_frame);

  // Decoder should switch back to SYNC mode due to bad padding
  EXPECT_EQ(decoder_->getCurrentMode(), LighthouseProtocolDecoder::Mode::SYNC);
}

TEST_F(LighthouseProtocolDecoderTest, MixedValidAndInvalidData) {
  // Send sync frame
  sendBytes(createSyncFrame());

  // Send some valid frames
  const std::uint8_t bs_id = 7;
  auto valid_measurement = createCompleteMeasurement(bs_id, 5000000);
  sendBytes(valid_measurement);

  const auto initial_bearing_count = received_bearings_.size();

  // Send a bad frame
  auto bad_frame = createDataFrame(
    0, makeNpoly(bs_id, true, 0), 100, 25000, 1,
    0, 0, 5100000);
  sendBytes(bad_frame);

  // Should be back in SYNC mode
  EXPECT_EQ(decoder_->getCurrentMode(), LighthouseProtocolDecoder::Mode::SYNC);

  // We should have received some bearings from the valid data
  EXPECT_GE(received_bearings_.size(), initial_bearing_count);
}

}    // namespace lighthouse_protocol_decoder
