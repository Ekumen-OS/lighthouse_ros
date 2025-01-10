#!/bin/python3

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

import sys
from lighthouse_ros.logger_proxy import LoggerProxy
from lighthouse_ros.frame_decoders import SyncFrameDecoder, DataFrameDecoder
from lighthouse_ros.types import ByteBuffer, DataFrameContents
from typing import Callable, Optional

import time
import serial


class FakeLogger(LoggerProxy):
    def __init__(self):
        """Initialize the logger."""
        super().__init__()

    def info(self, msg: str):
        """Log an info message."""
        print(f"INFO: {msg}")

    def warning(self, msg: str):
        """Log a warning message."""
        print(f"WARNING: {msg}")

    def error(self, msg: str):
        """Log an error message."""
        print(f"ERROR: {msg}")

    def debug(self, msg: str):
        """Log a debug message."""
        print(f"DEBUG: {msg}")


class LighthouseProtocolStreamPacer:
    """Decoder for the lighthouse."""

    MODE_SYNC: int = 0
    MODE_DATA: int = 1

    def __init__(
        self,
        logger: LoggerProxy,
        raw_frame_callback: Optional[Callable[[ByteBuffer], None]] = None,
    ):
        """Initialize the decoder."""
        self.__logger = logger
        self.current_mode = self.MODE_SYNC
        self.sync_frame_decoder = SyncFrameDecoder(
            callback=self.sync_frame_detected_callback,
            logger=self.__logger,
        )
        self.data_frame_decoder = None

        self.__latest_sync_frame_timestamp = None
        self.__latest_frame_timestamp = None
        self.__raw_frame_callback = raw_frame_callback

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
        if not good_sync:
            self.__logger.warning("Frame sync lost, switching back to sync mode...")
            self.current_mode = self.MODE_SYNC
            self.sync_frame_decoder = SyncFrameDecoder(
                callback=self.sync_frame_detected_callback,
                logger=self.__logger,
            )
            self.data_frame_decoder = None
        else:
            # Forward the data to the application
            self.forward_data(
                timestamp=frame_data.timestamp,
                raw_buffer=frame_data.raw_buffer,
            )

    def forward_data(
        self,
        timestamp: int,
        raw_buffer: ByteBuffer,
    ):
        """Forward the data to the application at a realistic pace."""
        # if more than 0.5 seconds have passed since the last sync frame, send a fake sync frame
        if self.__latest_sync_frame_timestamp is None:
            # force the first sync frame to be sent
            sync_frame_delta_t = 1.0
        else:
            sync_frame_delta_t = self.timestamp_to_seconds(
                timestamp, self.__latest_sync_frame_timestamp
            )
        if sync_frame_delta_t > 0.5:
            self.__latest_sync_frame_timestamp = timestamp
            self.__raw_frame_callback(self[255] * 12)

        # if we don't have a timestamp for the last frame, set it to the current timestamp
        if self.__latest_frame_timestamp is None:
            self.__latest_frame_timestamp = timestamp
        # sleep for the time between the last frame and the current frame and then send
        # the data through the callback
        data_frame_delta_t = self.timestamp_to_seconds(
            timestamp, self.__latest_frame_timestamp
        )
        time.sleep(data_frame_delta_t)
        self.__latest_frame_timestamp = timestamp
        self.__raw_frame_callback(raw_buffer)

    def timestamp_to_seconds(self, current: int, previous: int) -> float:
        """Convert a timestamp difference to seconds."""
        # do the calculation considering the difference with overflow, and the timestamp resolution
        self.__logger.error("MISSING CODE!!! RETURNING 0.0!!!")
        return 0.0


def process_file(filename, pseudo_tty_dev):
    """Process a file."""
    # open the file in binary read mode
    try:
        with serial.Serial(pseudo_tty_dev, 230400) as ser:

            def raw_frame_callback(raw_buffer: ByteBuffer):
                # forward data through pseudo-tty
                ser.write(bytes(raw_buffer))

            decoder = LighthouseProtocolStreamPacer(
                logger=FakeLogger(),
                raw_frame_callback=raw_frame_callback,
            )

            with open(filename, "rb") as file:
                # read the file as a byte array
                data = file.read()
                # process each byte in the file using the pacer
                for byte in data:
                    decoder.process_byte(byte)

    except FileNotFoundError:
        print(f"File {filename} not found.")
    except Exception as e:
        print(f"Exception caught: {e}")


def main():
    # define a single argument with the name of the file to process
    if len(sys.argv) != 3:
        print("Usage: file_parser_node.py <filename> <pseudo-tty>")
        return 1
    process_file(sys.argv[1], sys.argv[2])
    return 0


if __name__ == "__main__":
    exit(main())
