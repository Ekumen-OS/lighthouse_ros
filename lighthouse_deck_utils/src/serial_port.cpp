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

#include "lighthouse_deck_utils/serial_port.hpp"

#include <future>
#include <stdexcept>
#include <system_error>

using spb = asio::serial_port_base;

namespace lighthouse_deck_utils
{

SerialPort::SerialPort() = default;

SerialPort::~SerialPort() {close();}

bool SerialPort::open(
  const std::string & port_path,
  const PortConfiguration & config)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (serial_port_ && serial_port_->is_open()) {
    return false;
  }

  try {
    serial_port_ = std::make_unique<asio::serial_port>(io_context_, port_path);

    applyConfiguration(config);

    io_thread_ = std::make_unique<std::thread>(
      [this]() {
        asio::io_context::work work(io_context_);
        io_context_.run();
      });

    startAsyncRead();

    return true;
  } catch (const std::system_error & e) {
    serial_port_.reset();
    return false;
  }
}

void SerialPort::close()
{
  if (serial_port_ && serial_port_->is_open()) {
    io_context_.stop();

    if (io_thread_->joinable()) {
      io_thread_->join();
    }

    std::error_code ec;
    serial_port_->close(ec);

    serial_port_.reset();
    io_thread_.reset();

    io_context_.restart();
  }
}

bool SerialPort::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return internalIsOpen();
}

bool SerialPort::internalIsOpen() const
{
  return serial_port_ && serial_port_->is_open();
}

bool SerialPort::setConfiguration(const PortConfiguration & config)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!internalIsOpen()) {
    return false;
  }

  try {
    applyConfiguration(config);
    return true;
  } catch (const std::system_error & e) {
    return false;
  }
}

bool SerialPort::send(const uint8_t * data, std::size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!internalIsOpen()) {
    return false;
  }

  auto promise = std::make_shared<std::promise<std::error_code>>();
  auto future = promise->get_future();

  asio::async_write(
    *serial_port_, asio::buffer(data, size),
    [promise](const std::error_code & error, std::size_t bytes_transferred) {
      (void)bytes_transferred;
      promise->set_value(error);
    });

  std::error_code error = future.get();  // block until done
  return !error;
}

void SerialPort::sendBreak()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!internalIsOpen()) {
    return;
  }

  std::error_code ec;
  serial_port_->send_break(ec);
}

void SerialPort::setCallback(ReceiveCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  receive_callback_ = std::move(callback);
}

void SerialPort::startAsyncRead()
{
  if (!internalIsOpen()) {
    return;
  }

  serial_port_->async_read_some(
    asio::buffer(read_buffer_),
    [this](const std::error_code & error, std::size_t bytes_transferred) {
      if (!error && bytes_transferred > 0) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (receive_callback_) {
            receive_callback_(read_buffer_.data(), bytes_transferred);
          }
        }

        startAsyncRead();
      } else if (error == asio::error::operation_aborted) {
        return;
      } else {
        startAsyncRead();
      }
    });
}

void SerialPort::applyConfiguration(const PortConfiguration & config)
{
  if (!serial_port_) {
    throw std::runtime_error("Serial port not initialized");
  }

  serial_port_->set_option(
    asio::serial_port_base::baud_rate(
      static_cast<unsigned int>(config.baud_rate)));

  serial_port_->set_option(
    asio::serial_port_base::character_size(
      static_cast<unsigned int>(config.data_bits)));

  switch (config.stop_bits) {
    case StopBits::ONE:
      serial_port_->set_option(
        asio::serial_port_base::stop_bits(
          asio::serial_port_base::stop_bits::one));
      break;
    case StopBits::ONE_POINT_FIVE:
      serial_port_->set_option(
        asio::serial_port_base::stop_bits(
          asio::serial_port_base::stop_bits::onepointfive));
      break;
    case StopBits::TWO:
      serial_port_->set_option(
        asio::serial_port_base::stop_bits(
          asio::serial_port_base::stop_bits::two));
      break;
  }

  switch (config.parity) {
    case Parity::NONE:
      serial_port_->set_option(
        asio::serial_port_base::parity(asio::serial_port_base::parity::none));
      break;
    case Parity::ODD:
      serial_port_->set_option(
        asio::serial_port_base::parity(asio::serial_port_base::parity::odd));
      break;
    case Parity::EVEN:
      serial_port_->set_option(
        asio::serial_port_base::parity(asio::serial_port_base::parity::even));
      break;
  }

  switch (config.flow_control) {
    case FlowControl::NONE:
      serial_port_->set_option(
        asio::serial_port_base::flow_control(
          asio::serial_port_base::flow_control::none));
      break;
    case FlowControl::SOFTWARE:
      serial_port_->set_option(
        asio::serial_port_base::flow_control(
          asio::serial_port_base::flow_control::software));
      break;
    case FlowControl::HARDWARE:
      serial_port_->set_option(
        asio::serial_port_base::flow_control(
          asio::serial_port_base::flow_control::hardware));
      break;
  }
}

}  // namespace lighthouse_deck_utils
