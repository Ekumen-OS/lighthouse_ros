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

import zlib
import bitarray
import math


from lighthouse_ros.utils import OOTX_MAX_PAYLOAD_LENGTH
from lighthouse_ros.logger_proxy import LoggerProxy
from lighthouse_ros.types import ByteBuffer


class OOTXFrameDecoder:
    """Decoder for the OOTX protocol."""

    BUFFER_IS_NOT_A_FRAME = 0
    BUFFER_DECODED = 1
    OUTCOME_UNDEFINED = 2

    def __init__(self, logger: LoggerProxy):
        """Initialize the decoder."""
        self.__logger = logger
        self.__bit_buffer = []

    def process_slow_bit(self, slow_bit: int):
        """Process a slow bit."""
        self.__bit_buffer.append(
            slow_bit,
        )
        while len(self.__bit_buffer):
            retval = self.try_decode_frame(self.__bit_buffer)
            if retval == self.OUTCOME_UNDEFINED:
                # We do no changes to the buffer. We break the loop and wait for new
                # bits to arrive
                break
            elif retval == self.BUFFER_IS_NOT_A_FRAME:
                # remove bits from the buffer until a 0 comes out. This is because
                # the true preamble is 17 0s followed by a one, but our matcher
                # can only search for 16 0s followed by a one. We remove bits until we
                # find a candidate to be the 17th 0.
                while self.__bit_buffer and self.__bit_buffer.pop(0):
                    pass
            elif retval == self.BUFFER_DECODED:
                # empty the buffer, we successfully decoded the whole length of it.
                # TODO some unlikely scenarios with false positive detection from some previous
                # state of the buffer might end up with the buffer plus extra bits at the end,
                # so we might be discarding some bits that are part of the next frame.
                self.__bit_buffer = []

    def try_decode_frame(
        self,
        buffer: ByteBuffer,
    ):
        """Try to decode a frame from the buffer."""
        # the fist thing that the length needs to be a multiple of 17
        if len(buffer) % 17 != 0:
            return self.OUTCOME_UNDEFINED

        words_count = len(buffer) // 17

        # at least two 16 bit words needs to be present to decode
        # preamble and length
        if words_count < 2:
            return self.OUTCOME_UNDEFINED

        # every 17th bit needs to be 1
        sync_bits = [buffer[i * 17 + 16] for i in range(words_count)]

        if not all(sync_bits):
            return self.BUFFER_IS_NOT_A_FRAME

        data_bits = [buffer[(i * 17) : (i * 17 + 16)] for i in range(words_count)]

        # the first word is the preamble, needs to be all zeros
        preamble = self.seq_to_int(data_bits[0])
        if preamble != 0:
            return self.BUFFER_IS_NOT_A_FRAME

        # The second word is the length, it needs to be smaller or equal to 43
        length = self.seq_to_int(self.betole_bit_seq(data_bits[1]))
        if length > OOTX_MAX_PAYLOAD_LENGTH:
            return self.BUFFER_IS_NOT_A_FRAME

        # the full package includes the preamble, the length, the data and the
        # two words of the CRC32
        expected_words_count = 2 + int(math.ceil(length / 2)) + 2

        if words_count == 2:
            self.__logger.debug(
                "Corrections preamble detected, waiting for the rest..."
            )
        self.__logger.debug(
            "... received {} words out of {}. Good sync bits found: {}".format(
                words_count,
                expected_words_count,
                sync_bits,
            )
        )

        # Now that we know the length we wait until we have all the bits.
        # The length field measures the number of 16 bit words
        if words_count < expected_words_count:
            return self.OUTCOME_UNDEFINED

        data = data_bits[2:-2]

        upper_crc32 = self.seq_to_int(self.betole_bit_seq(data_bits[-1]))
        lower_crc32 = self.seq_to_int(self.betole_bit_seq(data_bits[-2]))
        transmitted_crc32 = (upper_crc32 << 16) + lower_crc32

        # If the length is odd, there's a padding byte of the payload that ensures the
        # transmitted crc is aligned on a word boundary. This padding needs to be removed
        # before calculating the crc32
        payload = b"".join(bitarray.bitarray(word).tobytes() for word in data)[0:length]

        if self.calculate_payload_crc32(payload) != transmitted_crc32:
            self.__logger.warning("CRC32 mismatch while trying to decode OOTX frame")
            return self.BUFFER_IS_NOT_A_FRAME

        self.__logger.info("OOTX corrections package decoded!")
        return self.BUFFER_DECODED

    def seq_to_int(self, seq):
        """Convert a sequence of bits to an integer assuming MSB first."""
        # convert a sequence of bits to an integer assuming MSB first
        value = sum([bit << i for i, bit in enumerate(seq[::-1])])
        return value

    def betole_bit_seq(self, data_seq):
        """Convert a sequence of bits from big endian to little endian."""
        # convert a sequence of bits from big endian to little endian
        return data_seq[8:16] + data_seq[0:8]

    def calculate_payload_crc32(self, payload):
        """Check the CRC32 of the payload."""
        return zlib.crc32(payload)
