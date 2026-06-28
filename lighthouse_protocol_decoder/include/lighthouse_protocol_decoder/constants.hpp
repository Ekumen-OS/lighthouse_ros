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

#ifndef LIGHTHOUSE_PROTOCOL_DECODER__CONSTANTS_HPP_
#define LIGHTHOUSE_PROTOCOL_DECODER__CONSTANTS_HPP_

#include <array>
#include <cstdint>
#include <map>

namespace lighthouse_protocol_decoder
{

/// Number of sensors in the lighthouse system
constexpr std::size_t kPulseProcessorNSensors = 4;

/// Maximum number of base stations supported
constexpr std::size_t kDeckLighthouseMaxNBs = 4;

/// 24-bit timestamp counter mask
constexpr std::uint32_t kTimestampCounterMask = (1 << 24) - 1;

/// Minimum ticks between slow bits (for filtering)
constexpr std::uint32_t kMinTicksBetweenSlowBits = (887000 / 2) * 8 / 10;

/// Timestamp clock frequency in Hz
constexpr double kTimestampClockFrequency = 24e6;

/// Maximum timestamp difference for sweep validation (in ticks)
/// Matches firmware's MAX_TICKS_SENSOR_TO_SENSOR (~0.42 ms at 24 MHz)
constexpr std::uint32_t kMaxTimestampDiffForSweep = 10000;

/// Maximum timestamp difference for block matching (in ticks)
constexpr std::uint32_t kMaxTimestampDiffForBlockMatch = 220000;

/// The cycle times from the Lighthouse base stations expressed in 24 MHz clock
/// (original values are in 48 MHz, divided by 2)
const std::map<std::uint8_t, double> kBasestationPeriods = {
  {1, 959000.0 / 2.0}, {2, 957000.0 / 2.0}, {3, 953000.0 / 2.0},
  {4, 949000.0 / 2.0}, {5, 947000.0 / 2.0}, {6, 943000.0 / 2.0},
  {7, 941000.0 / 2.0}, {8, 939000.0 / 2.0}, {9, 937000.0 / 2.0},
  {10, 929000.0 / 2.0}, {11, 919000.0 / 2.0}, {12, 911000.0 / 2.0},
  {13, 907000.0 / 2.0}, {14, 901000.0 / 2.0}, {15, 893000.0 / 2.0},
  {16, 887000.0 / 2.0},
};

/// Calculate the difference between two 24-bit timestamps with overflow
/// handling
/// @param a First timestamp
/// @param b Second timestamp
/// @return The difference (a - b) with proper wrap-around
inline std::uint32_t timestampDiff(std::uint32_t a, std::uint32_t b)
{
  return (kTimestampCounterMask + 1 + a - b) & kTimestampCounterMask;
}

/// Calculate the sum of two 24-bit timestamps with overflow handling
/// @param a First timestamp
/// @param b Second timestamp
/// @return The sum (a + b) with proper wrap-around
inline std::uint32_t timestampSum(std::uint32_t a, std::uint32_t b)
{
  return (a + b) & kTimestampCounterMask;
}

/// Check if the absolute difference between two timestamps is larger than a
/// limit
/// @param a First timestamp
/// @param b Second timestamp
/// @param limit The threshold to compare against
/// @return true if |a - b| > limit
inline bool timestampAbsDiffLargerThan(
  std::uint32_t a, std::uint32_t b,
  std::uint32_t limit)
{
  return timestampDiff(a + limit, b) > (limit * 2);
}

}    // namespace lighthouse_protocol_decoder

#endif  // LIGHTHOUSE_PROTOCOL_DECODER__CONSTANTS_HPP_
