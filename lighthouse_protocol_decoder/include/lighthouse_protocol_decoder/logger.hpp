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

#ifndef LIGHTHOUSE_PROTOCOL_DECODER__LOGGER_HPP_
#define LIGHTHOUSE_PROTOCOL_DECODER__LOGGER_HPP_

#include <memory>
#include <string>

namespace lighthouse_protocol_decoder
{

/// Interface for logging messages at different severity levels
class LoggerInterface
{
public:
  using Ptr = std::shared_ptr<LoggerInterface>;

  virtual ~LoggerInterface() = default;

  /// Log a debug message
  /// @param message The message to log
  virtual void debug(const std::string & message) = 0;

  /// Log an info message
  /// @param message The message to log
  virtual void info(const std::string & message) = 0;

  /// Log a warning message
  /// @param message The message to log
  virtual void warning(const std::string & message) = 0;

  /// Log an error message
  /// @param message The message to log
  virtual void error(const std::string & message) = 0;
};

/// Dummy logger implementation that does nothing (used for testing)
class NullLogger : public LoggerInterface
{
public:
  void debug([[maybe_unused]] const std::string & message) override {}
  void info([[maybe_unused]] const std::string & message) override {}
  void warning([[maybe_unused]] const std::string & message) override {}
  void error([[maybe_unused]] const std::string & message) override {}
};

}    // namespace lighthouse_protocol_decoder

#endif  // LIGHTHOUSE_PROTOCOL_DECODER__LOGGER_HPP_
