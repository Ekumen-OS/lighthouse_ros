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

#ifndef LIGHTHOUSE_STATION_MAPPER__COMMAND_QUEUE_HPP_
#define LIGHTHOUSE_STATION_MAPPER__COMMAND_QUEUE_HPP_

#include <functional>
#include <mutex>
#include <queue>

namespace lighthouse_station_mapper
{

/**
 * @brief Thread-safe command queue for passing work from UI thread to ROS thread.
 *
 * This class provides a simple thread-safe queue for function objects. Commands
 * are enqueued from the UI thread and processed in batch on the ROS thread.
 * The queue uses a mutex to protect against concurrent access.
 */
class CommandQueue
{
public:
  /**
   * @brief Enqueue a command to be executed later.
   *
   * Thread-safe. Typically called from the UI thread to schedule work
   * on the ROS thread.
   *
   * @param command Function to execute when the queue is processed.
   */
  void enqueue(std::function<void()> command);

  /**
   * @brief Process all queued commands and execute them.
   *
   * Thread-safe. Drains the queue and executes all pending commands
   * in FIFO order. Typically called from the ROS thread.
   */
  void process();

private:
  std::mutex mutex_;                           ///< Protects queue access.
  std::queue<std::function<void()>> queue_;   ///< FIFO queue of pending commands.
};

}  // namespace lighthouse_station_mapper

#endif  // LIGHTHOUSE_STATION_MAPPER__COMMAND_QUEUE_HPP_
