# Copyright 2024 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from lighthouse_ros.frame_decoders import SyncFrameDecoder, DataFrameDecoder
from lighthouse_ros.ootx_frame_decoder import OOTXFrameDecoder
from lighthouse_ros.sweep_processor import SweepProcessor
from lighthouse_ros.measurement_processor import MeasurementProcessor
from lighthouse_ros.types import DataFrameContents
from lighthouse_ros.types import SweepBlockRawData
from lighthouse_ros.types import SweepBlockBearings
from lighthouse_ros.utils import DECK_LIGHTHOUSE_MAX_N_BS
from lighthouse_ros.utils import MIN_TICKS_BETWEEN_SLOW_BITS
from lighthouse_ros.utils import PULSE_PROCESSOR_N_SENSORS
from lighthouse_ros.utils import timestamp_diff
from lighthouse_ros.utils import timestamp_abs_diff_larger_than
from lighthouse_ros.logger_proxy import LoggerProxy
from lighthouse_ros.types import BearingCallback

from typing import Optional


class LighthouseProtocolDecoder:
    """Decoder for the lighthouse."""

    MODE_SYNC: int = 0
    MODE_DATA: int = 1

    def __init__(
        self,
        logger: LoggerProxy,
        app_bearing_callback: Optional[BearingCallback] = None,
    ):
        """Initialize the decoder."""
        self.__logger = logger
        self.__app_bearing_callback = app_bearing_callback
        self.current_mode = self.MODE_SYNC
        self.sync_frame_decoder = SyncFrameDecoder(
            callback=self.sync_frame_detected_callback,
            logger=self.__logger,
        )
        self.data_frame_decoder = None
        self.sweep_decoder = SweepProcessor(
            callback=self.sweep_callback,
            logger=self.__logger,
        )
        self.measurement_processor = MeasurementProcessor(
            callback=self.measurement_callback,
            logger=self.__logger,
        )
        self.ootx_decoders = [None for _ in range(DECK_LIGHTHOUSE_MAX_N_BS)]
        self.prev_timestamp0 = [0 for _ in range(DECK_LIGHTHOUSE_MAX_N_BS)]

    def process_byte(self, byte: int) -> None:
        """Process a byte."""
        try:
            if self.current_mode == self.MODE_SYNC:
                self.sync_frame_decoder.process_byte(byte)
            elif self.current_mode == self.MODE_DATA:
                self.data_frame_decoder.process_byte(byte)
        except Exception as e:
            self.__logger.error(f"Exception caught: {e}")

    def sync_frame_detected_callback(self) -> None:
        """Process a sync frame detection."""
        self.__logger.info("Sync frame detected, switching into tracking mode...")
        self.current_mode = self.MODE_DATA
        self.sync_frame_decoder = None
        self.data_frame_decoder = DataFrameDecoder(
            self.__logger,
            self.dataframe_callback,
        )

    def dataframe_callback(
        self,
        good_sync: bool,
        frame_data: DataFrameContents,
    ):
        """Process a data frame."""
        self.__logger.debug(
            (
                "Sensor: {}  TS:{:06x}  Width:{:4x}  "
                + "Chan:{:4}({})  offset:{:-6d}  BeamWord:{:05x}\t"
            ).format(
                frame_data.sid,
                frame_data.timestamp,
                frame_data.width,
                frame_data.base_station_id(),
                frame_data.slow_bit() if frame_data.valid_npoly() else "-",
                frame_data.sync_offset,
                frame_data.beam_word,
            )
        )

        if not good_sync:
            self.__logger.warning("Frame sync lost, switching back to sync mode...")
            self.current_mode = self.MODE_SYNC
            self.sync_frame_decoder = SyncFrameDecoder(
                callback=self.sync_frame_detected_callback,
                logger=self.__logger,
            )
            self.data_frame_decoder = None
        else:
            self.sweep_decoder.process_frame(frame_data)
            # weird handing of the timestamp for the slow bit in
            # the cz handleCalibrationData function
            if (
                frame_data.valid_npoly()
                and (frame_data.base_station_id() < DECK_LIGHTHOUSE_MAX_N_BS)
                and (frame_data.sync_offset != 0)
            ):
                timestamp0 = timestamp_diff(
                    frame_data.timestamp, frame_data.sync_offset
                )
                if self.ootx_decoders[frame_data.base_station_id()] is None:
                    self.__logger.info(
                        "Creating new OOTX decoder for basestation base_station_id {}".format(
                            frame_data.base_station_id()
                        )
                    )
                    self.ootx_decoders[frame_data.base_station_id()] = OOTXFrameDecoder(
                        logger=self.__logger,
                    )
                    self.prev_timestamp0[frame_data.base_station_id()] = timestamp0
                prev_timestamp0 = self.prev_timestamp0[frame_data.base_station_id()]
                if timestamp_abs_diff_larger_than(
                    timestamp0, prev_timestamp0, MIN_TICKS_BETWEEN_SLOW_BITS
                ):
                    self.ootx_decoders[frame_data.base_station_id()].process_slow_bit(
                        frame_data.slow_bit()
                    )
                self.prev_timestamp0[frame_data.base_station_id()] = timestamp0

    def sweep_callback(self, sweep_contents: SweepBlockRawData) -> None:
        """Process a full sweep."""
        self.measurement_processor.process_block(sweep_contents)

    def measurement_callback(self, sensor_bearings: SweepBlockBearings) -> None:
        """Process a measurement."""
        self.__logger.debug(
            "Channel: {base_station_id}".format(
                base_station_id=sensor_bearings.base_station_id
            )
        )
        self.__logger.debug(
            "HW Timestamp: {hardware_timestamp}".format(
                hardware_timestamp=sensor_bearings.hardware_timestamp
            )
        )
        for i in range(PULSE_PROCESSOR_N_SENSORS):
            self.__logger.debug(
                "Sensor {sensor} Azimuth: {azimuth} Elevation: {elevation}".format(
                    sensor=i,
                    azimuth=sensor_bearings.sensor_angles[i].azimuth,
                    elevation=sensor_bearings.sensor_angles[i].elevation,
                )
            )
        if self.__app_bearing_callback is not None:
            self.__app_bearing_callback(sensor_bearings)
