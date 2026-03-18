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

#include "lighthouse_deck_hardware/lighthouse_deck_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

namespace lighthouse_deck_hardware {

LighthouseDeckHardware::LighthouseDeckHardware() {
  for (auto &base_station : base_station_data_) {
    for (auto &sensor : base_station.sensors) {
      sensor.azimuth = 0.0;
      sensor.elevation = 0.0;
    }
    base_station.timestamp = 0.0;
  }
  copy_to_read_buffer();
}

LighthouseDeckHardware::~LighthouseDeckHardware() {
  if (serial_port_) {
    try {
      serial_port_->close();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("LighthouseDeckHardware"),
                   "Error closing serial port: %s", e.what());
    }
  }
}

hardware_interface::CallbackReturn
LighthouseDeckHardware::on_init(const hardware_interface::HardwareInfo &info) {
  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Initializing Lighthouse Deck Hardware");
  if (hardware_interface::SensorInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("device") ==
      info_.hardware_parameters.end()) {
    RCLCPP_FATAL(rclcpp::get_logger("LighthouseDeckHardware"),
                 "Serial device parameter 'device' is required");
    return hardware_interface::CallbackReturn::ERROR;
  }
  device_ = info_.hardware_parameters.at("device");

  name_ = "lighthouse";
  if (info_.hardware_parameters.find("name") !=
      info_.hardware_parameters.end()) {
    name_ = info_.hardware_parameters.at("name");
  }

  baudrate_ = 230400;
  if (info_.hardware_parameters.find("baudrate") !=
      info_.hardware_parameters.end()) {
    try {
      baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("LighthouseDeckHardware"),
                   "Invalid baudrate parameter: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (std::find(VALID_BAUDRATES.begin(), VALID_BAUDRATES.end(), baudrate_) ==
      VALID_BAUDRATES.end()) {
    RCLCPP_FATAL(rclcpp::get_logger("LighthouseDeckHardware"),
                 "Invalid baudrate. Valid options are: 9600, 19200, 38400, "
                 "57600, 115200, 230400");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Lighthouse Deck Hardware initialized with name: %s, device: %s, "
              "baudrate: %d",
              name_.c_str(), device_.c_str(), baudrate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LighthouseDeckHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Configuring Lighthouse Deck Hardware");

  logger_adapter_ = std::make_shared<ROS2LoggerAdapter>(
      rclcpp::get_logger("LighthouseDeckHardware"));

  auto bearing_callback =
      [this](const lighthouse_protocol_decoder::SweepBlockBearings
                 &sensor_bearings) { this->bearing_callback(sensor_bearings); };

  protocol_decoder_ =
      std::make_unique<lighthouse_protocol_decoder::LighthouseProtocolDecoder>(
          bearing_callback, logger_adapter_);

  try {
    initialize_serial();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("LighthouseDeckHardware"),
                 "Failed to initialize serial device: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Lighthouse Deck Hardware configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LighthouseDeckHardware::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Cleaning up Lighthouse Deck Hardware");

  if (serial_port_) {
    try {
      serial_port_->close();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("LighthouseDeckHardware"),
                   "Error closing serial port: %s", e.what());
    }
    serial_port_.reset();
  }

  protocol_decoder_.reset();
  logger_adapter_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LighthouseDeckHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Activating Lighthouse Deck Hardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LighthouseDeckHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Deactivating Lighthouse Deck Hardware");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::string
LighthouseDeckHardware::format_base_station_id(size_t base_station) const {
  std::ostringstream oss;
  oss << std::setw(2) << std::setfill('0') << base_station;
  return oss.str();
}

std::vector<hardware_interface::StateInterface>
LighthouseDeckHardware::export_state_interfaces() {
  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Exporting state interfaces for Lighthouse Deck Hardware");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t sensor = 0; sensor < NUM_SENSORS; ++sensor) {
    for (size_t base_station = 0; base_station < NUM_BASE_STATIONS;
         ++base_station) {
      std::string interface_name = name_ + "_sensor_" + std::to_string(sensor) +
                                   "_base_" +
                                   format_base_station_id(base_station);

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          interface_name, "azimuth",
          &read_base_station_data_[base_station].sensors[sensor].azimuth));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          interface_name, "elevation",
          &read_base_station_data_[base_station].sensors[sensor].elevation));
    }
  }

  for (size_t base_station = 0; base_station < NUM_BASE_STATIONS;
       ++base_station) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        name_ + "_base_" + format_base_station_id(base_station), "timestamp",
        &read_base_station_data_[base_station].timestamp));
  }

  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

hardware_interface::return_type
LighthouseDeckHardware::read(const rclcpp::Time & /*time*/,
                             const rclcpp::Duration & /*period*/) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  copy_to_read_buffer();
  return hardware_interface::return_type::OK;
}

void LighthouseDeckHardware::initialize_serial() {
  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Initializing serial device %s with bootloader baudrate: %d",
              device_.c_str(), BOOTLOADER_BAUDRATE);

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

  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Resetting the bootloader state");
  serial_port_->sendBreak();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Sending command to enable bootloader UART");
  const uint8_t enable_uart_cmd = 0xBC;
  serial_port_->send(&enable_uart_cmd, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Sending command to boot into main firmware");
  const uint8_t boot_firmware_cmd = 0x00;
  serial_port_->send(&boot_firmware_cmd, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Switching to main firmware baudrate: %d", baudrate_);

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

  serial_port_->setCallback([this](const uint8_t *data, std::size_t length) {
    this->receive_callback(data, length);
  });

  RCLCPP_INFO(rclcpp::get_logger("LighthouseDeckHardware"),
              "Serial port initialized!");
}

void LighthouseDeckHardware::receive_callback(const uint8_t *data,
                                              std::size_t length) {
  for (size_t i = 0; i < length; ++i) {
    protocol_decoder_->processByte(data[i]);
  }
}

void LighthouseDeckHardware::copy_to_read_buffer() {
  read_base_station_data_ = base_station_data_;
}

void LighthouseDeckHardware::bearing_callback(
    const lighthouse_protocol_decoder::SweepBlockBearings &sensor_bearings) {
  std::lock_guard<std::mutex> lock(data_mutex_);

  uint8_t base_station_id = sensor_bearings.base_station_id;

  if (base_station_id >= NUM_BASE_STATIONS) {
    RCLCPP_WARN(rclcpp::get_logger("LighthouseDeckHardware"),
                "Received data for invalid base station ID: %d",
                base_station_id);
    return;
  }

  for (size_t sensor_idx = 0; sensor_idx < NUM_SENSORS; ++sensor_idx) {
    base_station_data_[base_station_id].sensors[sensor_idx].azimuth =
        sensor_bearings.sensor_angles[sensor_idx].azimuth;
    base_station_data_[base_station_id].sensors[sensor_idx].elevation =
        sensor_bearings.sensor_angles[sensor_idx].elevation;
  }

  auto now = std::chrono::steady_clock::now();
  auto duration = now.time_since_epoch();
  base_station_data_[base_station_id].timestamp =
      std::chrono::duration<double>(duration).count();
}

} // namespace lighthouse_deck_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(lighthouse_deck_hardware::LighthouseDeckHardware,
                       hardware_interface::SensorInterface)
