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

#include "lighthouse_station_mapper/command_queue.hpp"

#include <mutex>
#include <utility>

namespace lighthouse_station_mapper
{

void CommandQueue::enqueue(std::function<void()> command)
{
  std::lock_guard<std::mutex> lock(mutex_);
  queue_.push(std::move(command));
}

void CommandQueue::process()
{
  std::lock_guard<std::mutex> lock(mutex_);
  while (!queue_.empty()) {
    queue_.front()();
    queue_.pop();
  }
}

}  // namespace lighthouse_station_mapper
