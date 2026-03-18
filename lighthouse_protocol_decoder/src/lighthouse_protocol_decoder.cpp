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

#include "lighthouse_protocol_decoder/lighthouse_protocol_decoder.hpp"

#include <iomanip>
#include <sstream>

namespace lighthouse_protocol_decoder {

LighthouseProtocolDecoder::LighthouseProtocolDecoder(
    BearingCallback app_bearing_callback, LoggerInterface::Ptr logger)
    : current_mode_(Mode::SYNC),
      app_bearing_callback_(std::move(app_bearing_callback)),
      logger_(std::move(logger)) {

  // Initialize in SYNC mode
  sync_frame_decoder_ = std::make_unique<SyncFrameDecoder>(
      [this]() { syncFrameDetectedCallback(); }, logger_);

  // Create the sweep and measurement processors
  sweep_processor_ = std::make_unique<SweepProcessor>(
      [this](const SweepBlockRawData &sweep) { sweepCallback(sweep); },
      logger_);

  measurement_processor_ = std::make_unique<MeasurementProcessor>(
      [this](const SweepBlockBearings &bearings) {
        measurementCallback(bearings);
      },
      logger_);

  // Initialize timestamp tracking
  prev_timestamp0_.fill(0);
}

void LighthouseProtocolDecoder::processByte(std::uint8_t byte) {
  try {
    if (current_mode_ == Mode::SYNC) {
      if (sync_frame_decoder_) {
        sync_frame_decoder_->processByte(byte);
      }
    } else if (current_mode_ == Mode::DATA) {
      if (data_frame_decoder_) {
        data_frame_decoder_->processByte(byte);
      }
    }
  } catch (const std::exception &e) {
    if (logger_) {
      logger_->error(std::string("Exception caught: ") + e.what());
    }
  }
}

void LighthouseProtocolDecoder::reset() {
  current_mode_ = Mode::SYNC;
  sync_frame_decoder_ = std::make_unique<SyncFrameDecoder>(
      [this]() { syncFrameDetectedCallback(); }, logger_);
  data_frame_decoder_.reset();
  sweep_processor_->reset();
  measurement_processor_->reset();
  std::for_each(ootx_decoders_.begin(), ootx_decoders_.end(),
                [](auto &decoder) { decoder.reset(); });
  std::for_each(prev_timestamp0_.begin(), prev_timestamp0_.end(),
                [](auto &ts) { ts = 0; });
}

void LighthouseProtocolDecoder::syncFrameDetectedCallback() {
  if (logger_) {
    logger_->info("Sync frame detected, switching into tracking mode...");
  }

  // Switch to DATA mode
  current_mode_ = Mode::DATA;
  sync_frame_decoder_.reset();

  // Create data frame decoder
  data_frame_decoder_ = std::make_unique<DataFrameDecoder>(
      [this](bool good_sync, const DataFrameContents &frame) {
        dataframeCallback(good_sync, frame);
      },
      logger_);
}

void LighthouseProtocolDecoder::dataframeCallback(
    bool good_sync, const DataFrameContents &frame_data) {

  if (logger_) {
    std::ostringstream oss;
    oss << "Sensor: " << static_cast<int>(frame_data.sid) << "  TS:" << std::hex
        << std::setw(6) << std::setfill('0') << frame_data.timestamp
        << "  Width:" << std::setw(4) << frame_data.width << std::dec
        << "  Chan:" << std::setw(4)
        << static_cast<int>(frame_data.baseStationId()) << "("
        << (frame_data.validNpoly() ? std::to_string(frame_data.slowBit())
                                    : "-")
        << ")"
        << "  offset:" << std::setw(6) << std::left << frame_data.sync_offset
        << std::right << "  BeamWord:" << std::hex << std::setw(5)
        << std::setfill('0') << frame_data.beam_word << std::dec;
    logger_->debug(oss.str());
  }

  if (!good_sync) {
    if (logger_) {
      logger_->warning("Frame sync lost, switching back to sync mode...");
    }

    // Switch back to SYNC mode
    current_mode_ = Mode::SYNC;
    sync_frame_decoder_ = std::make_unique<SyncFrameDecoder>(
        [this]() { syncFrameDetectedCallback(); }, logger_);
    data_frame_decoder_.reset();
  } else {
    // Process the frame through the sweep processor
    sweep_processor_->processFrame(frame_data);

    // Handle OOTX slow bit processing
    if (frame_data.validNpoly() &&
        frame_data.baseStationId() < kDeckLighthouseMaxNBs &&
        frame_data.sync_offset != 0) {

      const std::uint32_t timestamp0 =
          timestampDiff(frame_data.timestamp, frame_data.sync_offset);

      const std::uint8_t bs_id = frame_data.baseStationId();

      // Create OOTX decoder for this base station if needed
      if (!ootx_decoders_[bs_id]) {
        if (logger_) {
          logger_->info("Creating new OOTX decoder for basestation " +
                        std::to_string(bs_id));
        }
        ootx_decoders_[bs_id] = std::make_unique<OOTXFrameDecoder>(logger_);
        prev_timestamp0_[bs_id] = timestamp0;
      }

      // Filter slow bits based on timing
      const std::uint32_t prev_timestamp0 = prev_timestamp0_[bs_id];
      if (timestampAbsDiffLargerThan(timestamp0, prev_timestamp0,
                                     kMinTicksBetweenSlowBits)) {
        ootx_decoders_[bs_id]->processSlowBit(frame_data.slowBit());
      }

      prev_timestamp0_[bs_id] = timestamp0;
    }
  }
}

void LighthouseProtocolDecoder::sweepCallback(
    const SweepBlockRawData &sweep_contents) {
  // Forward to measurement processor
  measurement_processor_->processBlock(sweep_contents);
}

void LighthouseProtocolDecoder::measurementCallback(
    const SweepBlockBearings &sensor_bearings) {

  if (logger_) {
    logger_->debug("Channel: " +
                   std::to_string(sensor_bearings.base_station_id));
    logger_->debug("HW Timestamp: " +
                   std::to_string(sensor_bearings.hardware_timestamp));

    for (std::size_t i = 0; i < kPulseProcessorNSensors; ++i) {
      std::ostringstream oss;
      oss << "Sensor " << i << " Azimuth: " << std::fixed
          << std::setprecision(2) << sensor_bearings.sensor_angles[i].azimuth
          << " Elevation: " << sensor_bearings.sensor_angles[i].elevation;
      logger_->debug(oss.str());
    }
  }

  // Forward to application callback if provided
  if (app_bearing_callback_) {
    app_bearing_callback_(sensor_bearings);
  }
}

} // namespace lighthouse_protocol_decoder
