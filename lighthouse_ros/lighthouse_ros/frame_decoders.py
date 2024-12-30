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

from lighthouse_ros.types import ByteBuffer
from lighthouse_ros.types import DataFrameContents
from lighthouse_ros.types import SyncFrameDetectedCallback
from lighthouse_ros.types import DataFrameCallback
from lighthouse_ros.logger_proxy import LoggerProxy

from typing import Optional


class SyncFrameDecoder:
    """Decoder for the sync frame."""

    def __init__(
        self,
        logger: LoggerProxy,
        callback: Optional[SyncFrameDetectedCallback] = None,
    ):
        """Initialize the decoder."""
        self.__logger = logger
        self.__frame_buffer = []
        self.__callback = callback

    def process_byte(
        self,
        byte: int,
    ) -> None:
        """Process a byte."""
        self.__frame_buffer.append(byte)
        if len(self.__frame_buffer) > 12:
            self.__frame_buffer.pop(0)
        # if the full frame is 0xff bytes, we found the sync frame
        if all([byte == 0xFF for byte in self.__frame_buffer]):
            if self.__callback is not None:
                self.__callback()


class DataFrameDecoder:
    """Decoder for the data frame."""

    def __init__(
        self,
        logger: LoggerProxy,
        callback: Optional[DataFrameCallback] = None,
    ):
        """Initialize the decoder."""
        self.__logger = logger
        self.__frame_buffer = []
        self.__good_sync = True
        self.__callback = callback

    def process_byte(
        self,
        byte: int,
    ) -> None:
        """Process a byte."""
        self.__frame_buffer.append(byte)
        if len(self.__frame_buffer) == 12:
            if all([byte == 0xFF for byte in self.__frame_buffer]):
                # Ignore sync frames
                self.__frame_buffer = []
                return
            frame_data = self.decode_frame(self.__frame_buffer)
            if frame_data.padding_1 != 0 or frame_data.padding_2 != 0:
                self.__logger.warning("Error: Bad padding value, resetting sync...")
                self.__good_sync = False
            self.__frame_buffer = []
            if self.__callback is not None:
                self.__callback(self.__good_sync, frame_data)

    def read_field(
        self,
        frame_buffer: ByteBuffer,
        start_bit: int,
        bit_width: int,
    ) -> int:
        """Read a field from the frame."""
        field_value = 0
        for i in range(0, bit_width):
            current_bit = start_bit + i
            n_byte = current_bit // 8
            n_bit = current_bit % 8
            field_value |= ((frame_buffer[n_byte] >> n_bit) & 1) << i
        return field_value

    def decode_frame(self, frame_buffer: ByteBuffer) -> DataFrameContents:
        """Decode the frame."""
        assert len(frame_buffer) == 12
        sid = self.read_field(frame_buffer, 0 + 0, 2)
        npoly = self.read_field(frame_buffer, 0 + 2, 6)
        width = self.read_field(frame_buffer, 0 + 8, 16)
        sync_offset = self.read_field(frame_buffer, 24 + 0, 17)
        padding_1 = self.read_field(frame_buffer, 24 + 17, 7)
        beam_word = self.read_field(frame_buffer, 48 + 0, 17)
        padding_2 = self.read_field(frame_buffer, 48 + 17, 7)
        timestamp = self.read_field(frame_buffer, 72 + 0, 24)

        # Offset is expressed in a 6 MHz clock, while the timestamp uses a 24 MHz clock.
        # update offset to a 24 MHz clock
        sync_offset = sync_offset * 4

        return DataFrameContents(
            sid=sid,
            npoly=npoly,
            width=width,
            sync_offset=sync_offset,
            padding_1=padding_1,
            beam_word=beam_word,
            padding_2=padding_2,
            timestamp=timestamp,
        )
