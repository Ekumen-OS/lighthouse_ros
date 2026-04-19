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

#ifndef LIGHTHOUSE_DECK_UTILS__SERIAL_PORT_HPP_
#define LIGHTHOUSE_DECK_UTILS__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <asio.hpp>

namespace lighthouse_deck_utils
{

class SerialPort
{
public:
  enum class BaudRate
  {
    BAUD_9600 = 9600,
    BAUD_19200 = 19200,
    BAUD_38400 = 38400,
    BAUD_57600 = 57600,
    BAUD_115200 = 115200,
    BAUD_230400 = 230400,
    BAUD_460800 = 460800,
    BAUD_921600 = 921600
  };

  enum class DataBits { FIVE = 5, SIX = 6, SEVEN = 7, EIGHT = 8 };

  enum class StopBits { ONE, ONE_POINT_FIVE, TWO };

  enum class Parity { NONE, ODD, EVEN };

  enum class FlowControl { NONE, SOFTWARE, HARDWARE };

  struct PortConfiguration
  {
    BaudRate baud_rate{BaudRate::BAUD_115200};
    DataBits data_bits{DataBits::EIGHT};
    StopBits stop_bits{StopBits::ONE};
    Parity parity{Parity::NONE};
    FlowControl flow_control{FlowControl::NONE};
  };

  using ReceiveCallback = std::function<void (const uint8_t *, std::size_t)>;

  SerialPort();
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;
  SerialPort(SerialPort &&) = delete;
  SerialPort & operator=(SerialPort &&) = delete;

  bool open(const std::string & port_path, const PortConfiguration & config);
  void close();
  bool isOpen() const;

  bool setConfiguration(const PortConfiguration & config);

  bool send(const uint8_t * data, std::size_t size);
  void sendBreak();
  void setCallback(ReceiveCallback callback);

private:
  void startAsyncRead();
  void applyConfiguration(const PortConfiguration & config);
  bool internalIsOpen() const;

  asio::io_context io_context_;
  std::unique_ptr<asio::serial_port> serial_port_;
  std::unique_ptr<std::thread> io_thread_;

  ReceiveCallback receive_callback_;
  mutable std::mutex mutex_;

  static constexpr std::size_t READ_BUFFER_SIZE = 1024;  //
  std::array<uint8_t, READ_BUFFER_SIZE> read_buffer_;
};

}  // namespace lighthouse_deck_utils

#endif  // LIGHTHOUSE_DECK_UTILS__SERIAL_PORT_HPP_
