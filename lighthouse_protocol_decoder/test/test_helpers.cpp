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

#include "test_helpers.hpp"

#include "lighthouse_protocol_decoder/constants.hpp"

namespace lighthouse_protocol_decoder
{
namespace test_helpers
{

std::vector<std::uint8_t>
createDataFrame(
  std::uint8_t sid, std::uint8_t npoly, std::uint16_t width,
  std::uint32_t sync_offset, std::uint8_t padding_1,
  std::uint32_t beam_word, std::uint8_t padding_2,
  std::uint32_t timestamp)
{
  std::vector<std::uint8_t> frame(12, 0x00);

  // Helper to write a field at a specific bit position
  auto writeField = [&frame](std::uint32_t value, std::size_t start_bit,
      std::size_t bit_width) {
      for (auto i = 0u; i < bit_width; ++i) {
        const auto current_bit = start_bit + i;
        const auto byte_index = current_bit / 8;
        const auto bit_index = current_bit % 8;

        const auto bit_value = static_cast<std::uint8_t>((value >> i) & 1);
        if (bit_value) {
          frame[byte_index] |= (1 << bit_index);
        }
      }
    };

  // Write fields according to the frame layout
  writeField(sid, 0, 2);           // sid: bits 0-1
  writeField(npoly, 2, 6);         // npoly: bits 2-7
  writeField(width, 8, 16);        // width: bits 8-23
  writeField(sync_offset, 24, 17);  // sync_offset: bits 24-40
  writeField(padding_1, 41, 7);    // padding_1: bits 41-47
  writeField(beam_word, 48, 17);   // beam_word: bits 48-64
  writeField(padding_2, 65, 7);    // padding_2: bits 65-71
  writeField(timestamp, 72, 24);   // timestamp: bits 72-95

  return frame;
}

DataFrameContents createDataFrameContents(
  std::uint8_t sid, std::uint8_t npoly,
  std::uint32_t timestamp,
  std::uint32_t sync_offset)
{
  DataFrameContents frame;
  frame.sid = sid;
  frame.npoly = npoly;
  frame.timestamp = timestamp;
  frame.sync_offset = sync_offset;
  frame.width = 100;
  frame.beam_word = 0;
  frame.padding_1 = 0;
  frame.padding_2 = 0;
  return frame;
}

std::uint8_t makeNpoly(
  std::uint8_t base_station_id, bool valid_npoly,
  std::uint8_t slow_bit)
{
  // npoly encoding:
  // baseStationId = (npoly / 2) + 1
  // Therefore: npoly = (baseStationId - 1) * 2 + slow_bit
  // bit 5: validity bit (0 = valid, 1 = invalid)
  std::uint8_t npoly = ((base_station_id - 1) * 2) + (slow_bit & 1);
  if (!valid_npoly) {
    npoly |= (1 << 5);  // Set bit 5 to mark as invalid
  }
  return npoly;
}

std::uint32_t makeBeamWord(std::uint8_t axis, std::uint8_t data_bit)
{
  // For simplicity, we encode axis in bit 0 and data_bit in bit 1
  std::uint32_t beam_word = 0;
  beam_word |= (axis & 1);
  beam_word |= ((data_bit & 1) << 1);
  return beam_word;
}

std::vector<std::uint8_t> createSyncFrame()
{
  return std::vector<std::uint8_t>(12, 0xFF);
}

std::vector<std::uint8_t> createSweepFrame(
  std::uint8_t sensor_id,
  std::uint8_t base_station_id,
  std::uint32_t timestamp,
  std::uint32_t sync_offset,
  bool valid_npoly)
{
  auto npoly = makeNpoly(base_station_id, valid_npoly, 0);
  auto beam_word = makeBeamWord(0, 0);
  auto frame = createDataFrame(
    sensor_id, npoly, 100, sync_offset / 4, 0,
    beam_word, 0, timestamp);
  return frame;
}

std::vector<std::uint8_t>
createCompleteMeasurement(
  std::uint8_t base_station_id,
  std::uint32_t base_timestamp)
{
  std::vector<std::uint8_t> data;

  // In lighthouse protocol:
  // - Exactly 3 sensors must have valid npoly
  // - Exactly 1 sensor must have non-zero sync_offset (reference sensor)
  // - Need TWO sweeps to form a matched pair for bearing calculation

  // First sweep (e.g., horizontal)
  // Sensor 0 is the reference with sync_offset
  auto sweep1_frame0 =
    createSweepFrame(0, base_station_id, base_timestamp, 100000, true);
  data.insert(data.end(), sweep1_frame0.begin(), sweep1_frame0.end());

  // Sensors 1 and 2 have valid npoly, no sync_offset
  auto sweep1_frame1 =
    createSweepFrame(1, base_station_id, base_timestamp + 10, 0, true);
  data.insert(data.end(), sweep1_frame1.begin(), sweep1_frame1.end());

  auto sweep1_frame2 =
    createSweepFrame(2, base_station_id, base_timestamp + 20, 0, true);
  data.insert(data.end(), sweep1_frame2.begin(), sweep1_frame2.end());

  // Sensor 3 has invalid npoly, no sync_offset
  auto sweep1_frame3 =
    createSweepFrame(3, base_station_id, base_timestamp + 30, 0, false);
  data.insert(data.end(), sweep1_frame3.begin(), sweep1_frame3.end());

  // Second sweep (e.g., vertical) - with larger offsets and later timestamp
  std::uint32_t timestamp2 = (base_timestamp + 5000) & kTimestampCounterMask;

  auto sweep2_frame0 =
    createSweepFrame(0, base_station_id, timestamp2, 200000, true);
  data.insert(data.end(), sweep2_frame0.begin(), sweep2_frame0.end());

  auto sweep2_frame1 =
    createSweepFrame(1, base_station_id, timestamp2 + 10, 0, true);
  data.insert(data.end(), sweep2_frame1.begin(), sweep2_frame1.end());

  auto sweep2_frame2 =
    createSweepFrame(2, base_station_id, timestamp2 + 20, 0, true);
  data.insert(data.end(), sweep2_frame2.begin(), sweep2_frame2.end());

  auto sweep2_frame3 =
    createSweepFrame(3, base_station_id, timestamp2 + 30, 0, false);
  data.insert(data.end(), sweep2_frame3.begin(), sweep2_frame3.end());

  return data;
}

std::vector<std::uint8_t>
createInterleavedMeasurements(
  const std::vector<std::uint8_t> & base_station_ids,
  std::uint32_t base_timestamp)
{
  std::vector<std::uint8_t> data;

  // Generate measurements for each basestation
  std::vector<std::vector<std::uint8_t>> all_measurements;
  for (auto bs_id : base_station_ids) {
    all_measurements.push_back(
      createCompleteMeasurement(bs_id, base_timestamp + bs_id * 50000));
  }

  // Interleave at the sweep level (4 sensors = 1 sweep)
  // Each measurement has 2 sweeps (8 frames total)
  // This simulates realistic data where basestations transmit in sequence
  // but we receive complete sweeps before switching to the next basestation
  const std::size_t frames_per_sweep = 4;       // 4 sensors per sweep
  const std::size_t sweeps_per_measurement = 2;  // 2 sweeps per measurement

  for (std::size_t sweep_idx = 0; sweep_idx < sweeps_per_measurement;
    ++sweep_idx)
  {
    for (auto & measurement : all_measurements) {
      const std::size_t start = sweep_idx * frames_per_sweep * 12;
      const std::size_t end = start + frames_per_sweep * 12;
      data.insert(
        data.end(), measurement.begin() + start,
        measurement.begin() + end);
    }
  }

  return data;
}

SweepBlockRawData
createSweepBlockRawData(
  std::uint8_t base_station_id, std::uint32_t timestamp,
  std::uint32_t offset0, std::uint32_t offset1,
  std::uint32_t offset2, std::uint32_t offset3)
{
  SweepBlockRawData block;
  block.base_station_id = base_station_id;
  block.timestamp = timestamp;
  block.sensors[0].normalized_offset = offset0;
  block.sensors[1].normalized_offset = offset1;
  block.sensors[2].normalized_offset = offset2;
  block.sensors[3].normalized_offset = offset3;
  return block;
}

}    // namespace test_helpers
}    // namespace lighthouse_protocol_decoder
