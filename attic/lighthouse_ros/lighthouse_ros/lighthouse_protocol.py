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

from serial.threaded import Protocol


class LighthouseProtocol(Protocol):
    """Interface between our decoder and the pyserial library."""

    def __init__(self, protocol_reader):
        """Initialize the protocol handler."""
        self.__reader = protocol_reader

    def connection_made(self, transport):
        """Store transport."""
        self.__transport = transport

    def connection_lost(self, exc):
        """Forget transport."""
        self.__transport = None

    def data_received(self, data):
        """Forward incoming data to the decoder."""
        try:
            for byte in data:
                self.__reader.process_byte(byte)
        except Exception:
            pass
