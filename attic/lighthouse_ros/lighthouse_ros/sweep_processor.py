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

from lighthouse_ros.types import DataFrameContents
from lighthouse_ros.types import SweepBlockRawData
from lighthouse_ros.types import SensorRawMeasurement
from lighthouse_ros.utils import timestamp_diff, timestamp_sum
from lighthouse_ros.logger_proxy import LoggerProxy
from lighthouse_ros.types import SweepCallback

from typing import Optional


class SweepProcessor:
    def __init__(
        self,
        logger: LoggerProxy,
        callback: Optional[SweepCallback] = None,
    ):
        """Initialize the sweep processor."""
        self.__callback = callback
        self.__block_frames = {}

    def process_frame(self, frame: DataFrameContents):
        """Process a frame."""
        latest_sensor = frame.sid
        latest_timestamp = frame.timestamp

        self.__block_frames[latest_sensor] = frame
        # remove any frames older than 0x10000 because they can't belong
        # to the same sweep
        sensors_to_remove = []
        for sensor, frame in self.__block_frames.items():
            if (
                timestamp_diff(latest_timestamp, frame.timestamp) > 0x10000
            ):  # TODO make this a proper documented constant
                sensors_to_remove.append(sensor)
        for sensor in sensors_to_remove:
            self.__block_frames.pop(sensor)
        if not self.validate_sweep():
            return
        if self.__callback is not None:
            self.__callback(self.complete_block_information())
        # reset the block frames buffer
        self.__block_frames = {}

    def validate_sweep(self) -> bool:
        """Validate if the set is a single basestation's sweep."""
        # if we have data from all four sensors, do some validation:
        # 1. three sensors must have valid npoly values
        # 2. all sensors must have the same base_station_id number, except the one missing the poly
        # 3. only one sensor has a valid offset measurement
        if len(self.__block_frames) != 4:
            return False
        valid_npolys = 0
        valid_offsets = 0
        channels_seen = set()
        for _, frame in self.__block_frames.items():
            valid_npolys += 1 if frame.valid_npoly() else 0
            valid_offsets += 1 if frame.sync_offset != 0 else 0
            if frame.valid_npoly():
                channels_seen.add(frame.base_station_id())
        if valid_npolys != 3 or valid_offsets != 1 or len(channels_seen) != 1:
            return False
        return True

    def complete_block_information(self):
        """Complete the block information and get the block data."""
        # find out the npoly and the offset of the block
        sweep_contents = SweepBlockRawData()
        reference_sensor_offset = None
        reference_sensor_timestamp = None
        for _, frame in self.__block_frames.items():
            if frame.valid_npoly():
                sweep_contents.base_station_id = frame.base_station_id()
            if frame.sync_offset != 0:
                reference_sensor_offset = frame.sync_offset
                reference_sensor_timestamp = frame.timestamp
        # complete the information in the frames
        for _, frame in self.__block_frames.items():
            sensor_measurement = SensorRawMeasurement()
            sensor_measurement.timestamp = frame.timestamp
            if frame.sync_offset != 0:
                sensor_measurement.normalized_offset = frame.sync_offset
            else:
                # calculate all offset relative to the offset sensor timestamp
                timestamp_delta = timestamp_diff(
                    frame.timestamp, reference_sensor_timestamp
                )
                sensor_measurement.normalized_offset = timestamp_sum(
                    reference_sensor_offset, timestamp_delta
                )
            sweep_contents.sensors[frame.sid] = sensor_measurement
        # TODO I have doubts about this; this will fail if the timestamp
        # overflows within the first and the last frame of the sweep
        sweep_contents.timestamp = min(
            [frame.timestamp for _, frame in self.__block_frames.items()]
        )
        return sweep_contents
