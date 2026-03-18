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

#ifndef LIGHTHOUSE_SHIELD_DRIVER__LIGHTHOUSE_SHIELD_DRIVER_NODE_HPP_
#define LIGHTHOUSE_SHIELD_DRIVER__LIGHTHOUSE_SHIELD_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <lighthouse_deck_msgs/msg/lighthouse_deck_measurement.hpp>
#include <lighthouse_protocol_decoder/lighthouse_protocol_decoder.hpp>
#include <lighthouse_protocol_decoder/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lighthouse_deck_utils/serial_port.hpp"

namespace lighthouse_deck_driver {

/// ROS2 Logger adapter for the lighthouse protocol decoder
class ROS2LoggerAdapter : public lighthouse_protocol_decoder::LoggerInterface {
public:
  explicit ROS2LoggerAdapter(rclcpp::Logger logger) : logger_(logger) {}

  void debug(const std::string &message) override {
    RCLCPP_DEBUG(logger_, "%s", message.c_str());
  }

  void info(const std::string &message) override {
    RCLCPP_INFO(logger_, "%s", message.c_str());
  }

  void warning(const std::string &message) override {
    RCLCPP_WARN(logger_, "%s", message.c_str());
  }

  void error(const std::string &message) override {
    RCLCPP_ERROR(logger_, "%s", message.c_str());
  }

private:
  rclcpp::Logger logger_;
};

class LighthouseDeckDriverNode : public rclcpp::Node {
public:
  explicit LighthouseDeckDriverNode(const rclcpp::NodeOptions &options);
  ~LighthouseDeckDriverNode() override;

private:
  void initializeSerial();
  void receiveCallback(const uint8_t *data, std::size_t length);
  void bearingCallback(
      const lighthouse_protocol_decoder::SweepBlockBearings &sensor_bearings);

  std::string device_;
  int baudrate_;
  std::string frame_id_;

  std::unique_ptr<lighthouse_deck_utils::SerialPort> serial_port_;
  std::unique_ptr<lighthouse_protocol_decoder::LighthouseProtocolDecoder>
      protocol_decoder_;
  std::shared_ptr<ROS2LoggerAdapter> logger_adapter_;
  rclcpp::Publisher<lighthouse_deck_msgs::msg::LighthouseDeckMeasurement>::
      SharedPtr bearings_publisher_;

  static constexpr int BOOTLOADER_BAUDRATE = 115200;
};

} // namespace lighthouse_deck_driver

#endif // LIGHTHOUSE_SHIELD_DRIVER__LIGHTHOUSE_SHIELD_DRIVER_NODE_HPP_
