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

#ifndef LIGHTHOUSE_DECK_HARDWARE__LIGHTHOUSE_DECK_HARDWARE_HPP_
#define LIGHTHOUSE_DECK_HARDWARE__LIGHTHOUSE_DECK_HARDWARE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "lighthouse_deck_utils/serial_port.hpp"
#include "lighthouse_protocol_decoder/datatypes.hpp"
#include "lighthouse_protocol_decoder/lighthouse_protocol_decoder.hpp"
#include "lighthouse_protocol_decoder/logger.hpp"

namespace lighthouse_deck_hardware
{

constexpr size_t NUM_BASE_STATIONS = 4;
constexpr size_t NUM_SENSORS = 4;
constexpr int BOOTLOADER_BAUDRATE = 115200;

const std::array<int, 6> VALID_BAUDRATES = {9600, 19200, 38400,
  57600, 115200, 230400};

/// ROS2 Logger Adapter for the lighthouse protocol decoder
class ROS2LoggerAdapter : public lighthouse_protocol_decoder::LoggerInterface
{
public:
  explicit ROS2LoggerAdapter(rclcpp::Logger logger)
  : logger_(logger) {}

  void debug(const std::string & message) override
  {
    RCLCPP_DEBUG(logger_, "%s", message.c_str());
  }

  void info(const std::string & message) override
  {
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  void warning(const std::string & message) override
  {
    RCLCPP_WARN(logger_, "%s", message.c_str());
  }

  void error(const std::string & message) override
  {
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  }

private:
  rclcpp::Logger logger_;
};

/// Lighthouse Deck Hardware Interface for ros2_control
///
/// This sensor interface provides azimuth and elevation angles for multiple
/// sensors from multiple base stations (beacons). Each sensor can see multiple
/// base stations, and we export the angles for each combination.
class LighthouseDeckHardware : public hardware_interface::SensorInterface
{
public:
  LighthouseDeckHardware();
  ~LighthouseDeckHardware() override;

  // Lifecycle interface methods
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  /// Initialize the serial port and bootloader sequence
  void initialize_serial();

  /// Serial port receive callback
  void receive_callback(const uint8_t * data, std::size_t length);

  /// Bearing callback from protocol decoder
  void bearing_callback(
    const lighthouse_protocol_decoder::SweepBlockBearings & sensor_bearings);

  /// Copy base_station_data_ to read_base_station_data_ (must be called with
  /// mutex held)
  void copy_to_read_buffer();

  /// Format base station ID as 2-digit zero-padded string
  std::string format_base_station_id(size_t base_station) const;

  struct SensorAngles
  {
    double azimuth{0.0};
    double elevation{0.0};
  };

  struct BaseStationData
  {
    std::array<SensorAngles, NUM_SENSORS> sensors;
    double timestamp{0.0};
  };

  std::string device_;
  int baudrate_;
  std::string name_;

  std::unique_ptr<lighthouse_deck_utils::SerialPort> serial_port_;
  std::unique_ptr<lighthouse_protocol_decoder::LighthouseProtocolDecoder>
  protocol_decoder_;
  std::shared_ptr<ROS2LoggerAdapter> logger_adapter_;

  // base_station_data_ is written by callback (protected by mutex)
  // read_base_station_data_ is read by ros2_control (copied in read())
  std::array<BaseStationData, NUM_BASE_STATIONS> base_station_data_;
  std::array<BaseStationData, NUM_BASE_STATIONS> read_base_station_data_;

  mutable std::mutex data_mutex_;
};

}  // namespace lighthouse_deck_hardware

#endif  // LIGHTHOUSE_DECK_HARDWARE__LIGHTHOUSE_DECK_HARDWARE_HPP_
