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

from dataclasses import dataclass, field
from typing import List, Callable

from lighthouse_ros.utils import PULSE_PROCESSOR_N_SENSORS
from lighthouse_ros.utils import npoly_is_valid
from lighthouse_ros.utils import npoly_channel
from lighthouse_ros.utils import npoly_slow_bit

ByteBuffer = List[int]


@dataclass
class DataFrameContents:
    sid: int = 0
    npoly: int = 0
    width: int = 0
    sync_offset: int = 0
    padding_1: int = 0
    beam_word: int = 0
    padding_2: int = 0
    timestamp: int = 0
    raw_data: ByteBuffer = field(default_factory=list)

    def valid_npoly(self) -> bool:
        """Check if the validity bit for the npoly value is set."""
        return npoly_is_valid(self.npoly)

    def slow_bit(self) -> int:
        """Get the slow bit from the npoly value."""
        return npoly_slow_bit(self.npoly)

    def base_station_id(self) -> int:
        """Get the base_station_id from the npoly value."""
        return npoly_channel(self.npoly)


@dataclass
class SensorRawMeasurement:
    normalized_offset: int = 0


@dataclass
class SweepBlockRawData:
    base_station_id: int = 0
    timestamp: int = 0
    sensors: List[SensorRawMeasurement] = field(
        default_factory=lambda: [
            SensorRawMeasurement() for i in range(PULSE_PROCESSOR_N_SENSORS)
        ]
    )


@dataclass
class SingleSensorBearing:
    azimuth: float = 0.0
    elevation: float = 0.0


@dataclass
class SweepBlockBearings:
    base_station_id: int = 0
    hardware_timestamp: int = 0
    sensor_angles: List[SingleSensorBearing] = field(
        default_factory=lambda: [
            SingleSensorBearing() for i in range(PULSE_PROCESSOR_N_SENSORS)
        ]
    )


DataFrameCallback = Callable[[bool, DataFrameContents], None]

SyncFrameDetectedCallback = Callable[[], None]

SweepCallback = Callable[[SweepBlockRawData], None]

BearingCallback = Callable[[SweepBlockBearings], None]
