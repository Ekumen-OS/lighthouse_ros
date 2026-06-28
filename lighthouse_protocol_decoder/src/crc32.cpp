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

#include "lighthouse_protocol_decoder/crc32.hpp"

#include <cppcrc.h>

namespace lighthouse_protocol_decoder
{

std::uint32_t calculateCRC32(const std::vector<std::uint8_t> & data)
{
  return CRC32::CRC32::calc(data.data(), data.size());
}

}    // namespace lighthouse_protocol_decoder
