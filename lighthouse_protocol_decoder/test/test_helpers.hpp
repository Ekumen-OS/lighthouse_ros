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

#ifndef LIGHTHOUSE_PROTOCOL_DECODER__TEST__TEST_HELPERS_HPP_
#define LIGHTHOUSE_PROTOCOL_DECODER__TEST__TEST_HELPERS_HPP_

#include <cstdint>
#include <vector>

#include "lighthouse_protocol_decoder/datatypes.hpp"

namespace lighthouse_protocol_decoder {
namespace test_helpers {

/// Create a raw data frame as a vector of bytes
/// @param sid Sensor ID (0-3)
/// @param npoly Polynomial value (includes channel, validity, slow bit)
/// @param width Width of the sweep in ticks
/// @param sync_offset Offset from sync to sweep (will be multiplied by 4)
/// @param padding_1 First padding field (should be 0)
/// @param beam_word Beam word containing sweep information
/// @param padding_2 Second padding field (should be 0)
/// @param timestamp Timestamp counter value (24-bit)
/// @return Vector of 12 bytes representing the data frame
std::vector<std::uint8_t>
createDataFrame(std::uint8_t sid, std::uint8_t npoly, std::uint16_t width,
                std::uint32_t sync_offset, std::uint8_t padding_1,
                std::uint32_t beam_word, std::uint8_t padding_2,
                std::uint32_t timestamp);

/// Create a DataFrameContents object for testing
/// @param sid Sensor ID (0-3)
/// @param npoly Polynomial value
/// @param timestamp Timestamp value
/// @param sync_offset Sync offset value (default 0)
/// @return DataFrameContents object
DataFrameContents createDataFrameContents(std::uint8_t sid, std::uint8_t npoly,
                                          std::uint32_t timestamp,
                                          std::uint32_t sync_offset = 0);

/// Create npoly value from base station ID, validity, and slow bit
/// @param base_station_id Base station ID (1-16)
/// @param valid_npoly If true, npoly is valid (bit 5 is clear)
/// @param slow_bit The slow bit value (0 or 1)
/// @return Encoded npoly value
/// @note baseStationId() returns (npoly / 2) + 1, so npoly = (bs_id - 1) * 2 +
/// slow_bit
std::uint8_t makeNpoly(std::uint8_t base_station_id, bool valid_npoly,
                       std::uint8_t slow_bit);

/// Create beam word encoding sweep axis and data bit
/// @param axis Sweep axis (0 = horizontal, 1 = vertical)
/// @param data_bit Data bit value
/// @return Encoded beam word
std::uint32_t makeBeamWord(std::uint8_t axis, std::uint8_t data_bit);

/// Create a sync frame (12 bytes of 0xFF)
/// @return Vector of 12 bytes with all set to 0xFF
std::vector<std::uint8_t> createSyncFrame();

/// Create a single sweep frame for a sensor
/// @param sensor_id Sensor ID (0-3)
/// @param base_station_id Base station ID (1-16)
/// @param timestamp Timestamp for the sweep
/// @param sync_offset Offset from sync (0 if not the reference sensor)
/// @param valid_npoly Whether this sensor has valid npoly
/// @return Vector of bytes representing the frame
std::vector<std::uint8_t> createSweepFrame(std::uint8_t sensor_id,
                                           std::uint8_t base_station_id,
                                           std::uint32_t timestamp,
                                           std::uint32_t sync_offset,
                                           bool valid_npoly);

/// Create a complete measurement sequence with all 4 sensors for one
/// basestation This creates TWO complete sweeps (a matched pair) to generate
/// bearing measurements
/// @param base_station_id Base station ID (1-16)
/// @param base_timestamp Base timestamp for the measurement
/// @return Vector of bytes representing complete measurement sequence (2
/// sweeps)
std::vector<std::uint8_t>
createCompleteMeasurement(std::uint8_t base_station_id,
                          std::uint32_t base_timestamp);

/// Create interleaved measurements from multiple basestations
/// @param base_station_ids Vector of base station IDs
/// @param base_timestamp Base timestamp for the measurements
/// @return Vector of bytes with interleaved data from all basestations
std::vector<std::uint8_t>
createInterleavedMeasurements(const std::vector<std::uint8_t> &base_station_ids,
                              std::uint32_t base_timestamp);

/// Create a SweepBlockRawData for testing
/// @param base_station_id Base station ID
/// @param timestamp Timestamp
/// @param offset0 Offset for sensor 0
/// @param offset1 Offset for sensor 1
/// @param offset2 Offset for sensor 2
/// @param offset3 Offset for sensor 3
/// @return SweepBlockRawData object
SweepBlockRawData
createSweepBlockRawData(std::uint8_t base_station_id, std::uint32_t timestamp,
                        std::uint32_t offset0, std::uint32_t offset1,
                        std::uint32_t offset2, std::uint32_t offset3);

} // namespace test_helpers
} // namespace lighthouse_protocol_decoder

#endif // LIGHTHOUSE_PROTOCOL_DECODER__TEST__TEST_HELPERS_HPP_
