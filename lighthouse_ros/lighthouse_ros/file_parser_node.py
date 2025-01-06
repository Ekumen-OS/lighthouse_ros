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
from lighthouse_ros.lighthouse_protocol_decoder import (
    LighthouseProtocolDecoder,
)
from lighthouse_ros.logger_proxy import LoggerProxy


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


def process_file(filename):
    """Process a file."""
    # open the file in binary read mode
    try:
        with open(filename, "rb") as file:
            # read the file as a byte array
            data = file.read()
            # iterate over the byte array
            decoder = LighthouseProtocolDecoder(
                logger=FakeLogger(),
            )
            for byte in data:
                decoder.process_byte(byte)
    except FileNotFoundError:
        print(f"File {filename} not found.")


def main():
    # define a single argument with the name of the file to process
    if len(sys.argv) != 2:
        print("Usage: file_parser_node.py <filename>")
        return 1
    process_file(sys.argv[1])
    return 0


if __name__ == "__main__":
    exit(main())
