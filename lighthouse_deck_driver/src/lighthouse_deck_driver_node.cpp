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

#include "lighthouse_deck_driver/lighthouse_deck_driver_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <sstream>
#include <thread>

#include <cmath>

#include <lighthouse_deck_msgs/msg/lighthouse_deck_measurement.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace lighthouse_deck_driver
{

LighthouseDeckDriverNode::LighthouseDeckDriverNode(
  const rclcpp::NodeOptions & options)
: Node("lighthouse_deck_driver", options)
{
  RCLCPP_INFO(get_logger(), "Starting Lighthouse Shield Driver node");

  rcl_interfaces::msg::ParameterDescriptor device_desc;
  device_desc.description =
    "Serial device path for the Lighthouse Shield (e.g., /dev/ttyUSB0)";
  declare_parameter("device", "", device_desc);

  const std::array<int, 6> valid_baudrates{9600, 19200, 38400,
    57600, 115200, 230400};
  std::ostringstream valid_baudrates_stream;
  for (size_t i = 0; i < valid_baudrates.size(); ++i) {
    if (i > 0) {
      valid_baudrates_stream << ", ";
    }
    valid_baudrates_stream << valid_baudrates[i];
  }
  const std::string valid_baudrates_list = valid_baudrates_stream.str();

  rcl_interfaces::msg::ParameterDescriptor baudrate_desc;
  baudrate_desc.description =
    "Baudrate for serial communication with the Lighthouse Shield. Valid "
    "values are: " +
    valid_baudrates_list;
  declare_parameter("baudrate", 230400, baudrate_desc);

  rcl_interfaces::msg::ParameterDescriptor frame_id_desc;
  frame_id_desc.description =
    "Frame ID to use in published sensor measurement messages";
  declare_parameter("frame_id", "lighthouse_deck", frame_id_desc);

  device_ = get_parameter("device").as_string();
  baudrate_ = get_parameter("baudrate").as_int();
  frame_id_ = get_parameter("frame_id").as_string();

  if (device_.empty()) {
    RCLCPP_FATAL(get_logger(), "Serial device parameter is required");
    throw std::runtime_error("Serial device parameter is required");
  }

  if (std::find(valid_baudrates.begin(), valid_baudrates.end(), baudrate_) ==
    valid_baudrates.end())
  {
    RCLCPP_FATAL(
      get_logger(), "Invalid baudrate. Valid options are: %s",
      valid_baudrates_list.c_str());
    throw std::runtime_error("Invalid baudrate");
  }

  RCLCPP_INFO(get_logger(), "Device: %s", device_.c_str());
  RCLCPP_INFO(get_logger(), "Baudrate: %d", baudrate_);
  RCLCPP_INFO(get_logger(), "Frame ID: %s", frame_id_.c_str());

  bearings_publisher_ =
    create_publisher<lighthouse_deck_msgs::msg::LighthouseDeckMeasurement>(
    "lighthouse", rclcpp::QoS(10).best_effort());

  logger_adapter_ = std::make_shared<ROS2LoggerAdapter>(get_logger());

  auto bearing_callback =
    [this](const lighthouse_protocol_decoder::SweepBlockBearings
      & sensor_bearings) {this->bearingCallback(sensor_bearings);};

  protocol_decoder_ =
    std::make_unique<lighthouse_protocol_decoder::LighthouseProtocolDecoder>(
    bearing_callback, logger_adapter_);

  initializeSerial();

  RCLCPP_INFO(get_logger(), "Lighthouse Deck Driver node started");
}

LighthouseDeckDriverNode::~LighthouseDeckDriverNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down Lighthouse Deck Driver node");
  if (serial_port_) {
    try {
      serial_port_->close();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Error closing serial port: %s", e.what());
    }
  }
}

void LighthouseDeckDriverNode::initializeSerial()
{
  RCLCPP_INFO(
    get_logger(),
    "Initializing serial device %s with bootloader baudrate: %d",
    device_.c_str(), BOOTLOADER_BAUDRATE);

  try {
    serial_port_ = std::make_unique<lighthouse_deck_utils::SerialPort>();

    lighthouse_deck_utils::SerialPort::PortConfiguration bootloader_config;
    bootloader_config.baud_rate =
      static_cast<lighthouse_deck_utils::SerialPort::BaudRate>(
      BOOTLOADER_BAUDRATE);
    bootloader_config.data_bits =
      lighthouse_deck_utils::SerialPort::DataBits::EIGHT;
    bootloader_config.stop_bits =
      lighthouse_deck_utils::SerialPort::StopBits::ONE;
    bootloader_config.parity = lighthouse_deck_utils::SerialPort::Parity::NONE;
    bootloader_config.flow_control =
      lighthouse_deck_utils::SerialPort::FlowControl::NONE;

    if (!serial_port_->open(device_, bootloader_config)) {
      throw std::runtime_error("Failed to open serial port");
    }

    RCLCPP_INFO(get_logger(), "Resetting the bootloader state");
    serial_port_->sendBreak();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    RCLCPP_INFO(get_logger(), "Sending command to enable bootloader UART");
    const uint8_t enable_uart_cmd = 0xBC;
    serial_port_->send(&enable_uart_cmd, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    RCLCPP_INFO(get_logger(), "Sending command to boot into main firmware");
    const uint8_t boot_firmware_cmd = 0x00;
    serial_port_->send(&boot_firmware_cmd, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    RCLCPP_INFO(
      get_logger(), "Switching to main firmware baudrate: %d",
      baudrate_);

    lighthouse_deck_utils::SerialPort::PortConfiguration main_config;
    main_config.baud_rate =
      static_cast<lighthouse_deck_utils::SerialPort::BaudRate>(baudrate_);
    main_config.data_bits = lighthouse_deck_utils::SerialPort::DataBits::EIGHT;
    main_config.stop_bits = lighthouse_deck_utils::SerialPort::StopBits::ONE;
    main_config.parity = lighthouse_deck_utils::SerialPort::Parity::NONE;
    main_config.flow_control =
      lighthouse_deck_utils::SerialPort::FlowControl::NONE;

    if (!serial_port_->setConfiguration(main_config)) {
      throw std::runtime_error("Failed to set serial port configuration");
    }

    serial_port_->setCallback(
      [this](const uint8_t * data, std::size_t length) {
        this->receiveCallback(data, length);
      });

    RCLCPP_INFO(get_logger(), "Serial port initialized!");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_logger(), "Failed to initialize serial device: %s",
      e.what());
    throw;
  }
}

void LighthouseDeckDriverNode::receiveCallback(
  const uint8_t * data,
  std::size_t length)
{
  for (size_t i = 0; i < length; ++i) {
    protocol_decoder_->processByte(data[i]);
  }
}

void LighthouseDeckDriverNode::bearingCallback(
  const lighthouse_protocol_decoder::SweepBlockBearings & sensor_bearings)
{
  static int32_t measurements_count = 0;
  measurements_count++;

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Bearing measurements published: %d",
    measurements_count);

  auto msg = lighthouse_deck_msgs::msg::LighthouseDeckMeasurement();
  msg.header.stamp = now();
  msg.header.frame_id = frame_id_;

  // Publish data from a single base station
  // Each array contains one element for this base station
  // convert the angles from degrees to radians for the message

  const auto kDegToRad = M_PI / 180.0;

  msg.station_id = static_cast<int32_t>(sensor_bearings.base_station_id);
  msg.azimuth_0 = sensor_bearings.sensor_angles[0].azimuth * kDegToRad;
  msg.azimuth_1 = sensor_bearings.sensor_angles[1].azimuth * kDegToRad;
  msg.azimuth_2 = sensor_bearings.sensor_angles[2].azimuth * kDegToRad;
  msg.azimuth_3 = sensor_bearings.sensor_angles[3].azimuth * kDegToRad;
  msg.elevation_0 = sensor_bearings.sensor_angles[0].elevation * kDegToRad;
  msg.elevation_1 = sensor_bearings.sensor_angles[1].elevation * kDegToRad;
  msg.elevation_2 = sensor_bearings.sensor_angles[2].elevation * kDegToRad;
  msg.elevation_3 = sensor_bearings.sensor_angles[3].elevation * kDegToRad;

  bearings_publisher_->publish(msg);
}

}  // namespace lighthouse_deck_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(
  lighthouse_deck_driver::LighthouseDeckDriverNode)
