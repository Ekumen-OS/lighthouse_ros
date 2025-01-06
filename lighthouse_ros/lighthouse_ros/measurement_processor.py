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

import math
from lighthouse_ros.types import SweepBlockBearings
from lighthouse_ros.types import SingleSensorBearing
from lighthouse_ros.types import SweepBlockRawData
from lighthouse_ros.utils import PERIODS
from lighthouse_ros.utils import timestamp_diff
from lighthouse_ros.utils import PULSE_PROCESSOR_N_SENSORS
from lighthouse_ros.logger_proxy import LoggerProxy
from lighthouse_ros.types import BearingCallback

from typing import Optional


class MeasurementProcessor:
    def __init__(
        self,
        logger: LoggerProxy,
        callback: Optional[BearingCallback] = None,
    ):
        """Initialize the measurement processor."""
        self.__logger = logger
        self.__callback = callback
        self.__per_channel_buffer = {}

    def process_block(self, sweep_contents: SweepBlockRawData):
        """Process a block."""
        base_station_id = sweep_contents.base_station_id
        if base_station_id not in self.__per_channel_buffer:
            self.__per_channel_buffer[base_station_id] = []
        self.__per_channel_buffer[base_station_id].append(sweep_contents)
        if len(self.__per_channel_buffer[base_station_id]) > 2:
            self.__per_channel_buffer[base_station_id].pop(0)
        current = self.__per_channel_buffer[base_station_id][-1]
        previous = self.__per_channel_buffer[base_station_id][0]
        if not self.blocks_are_matched_pair(current, previous):
            return
        # we have a matched pair of blocks, extract the measurements
        sensor_bearings = self.extract_measurements(current, previous, base_station_id)
        if self.__callback is not None:
            self.__callback(sensor_bearings)

    def blocks_are_matched_pair(self, current, previous):
        """Check if two consecutive blocks from a base station are a measurement pair."""
        if previous.sensors[0].normalized_offset > current.sensors[0].normalized_offset:
            return False
        block_delta_timestamp = timestamp_diff(current.timestamp, previous.timestamp)
        # 220000 ticks is around 180 degrees
        if (
            block_delta_timestamp > 220000
        ):  # TODO convert this into a proper documented constant
            return False
        return True

    def extract_measurements(self, current, previous, base_station_id):
        """Calculate azimuth and elevation from the matched pair of blocks."""
        sensor_bearings = SweepBlockBearings()
        sensor_bearings.base_station_id = base_station_id
        sensor_bearings.hardware_timestamp = current.timestamp

        channel_period = PERIODS[base_station_id]

        for i in range(PULSE_PROCESSOR_N_SENSORS):
            offset_0 = previous.sensors[i].normalized_offset
            offset_1 = current.sensors[i].normalized_offset

            phase_beam_0 = (offset_0 / channel_period) * 2.0 * math.pi
            phase_beam_1 = (offset_1 / channel_period) * 2.0 * math.pi

            azimuth_rad, elevation_rad = self.calculate_polar_bearing(
                phase_beam_0, phase_beam_1
            )

            azimuth_deg = math.degrees(azimuth_rad)
            elevation_deg = math.degrees(elevation_rad)

            sensor_bearings.sensor_angles[i] = SingleSensorBearing(
                azimuth=azimuth_deg,
                elevation=elevation_deg,
            )
        return sensor_bearings

    def calculate_polar_bearing(self, phase_beam_0, phase_beam_1):
        """Calculate the polar coordinates of the sensor relative to the base station."""
        azimuth = ((phase_beam_0 + phase_beam_1) / 2) - math.pi
        p = math.radians(60)
        beta = (phase_beam_1 - phase_beam_0) - math.radians(120)
        elevation = math.atan(math.sin(beta / 2) / math.tan(p / 2))
        return (azimuth, elevation)
