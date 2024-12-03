from dataclasses import dataclass
from enum import Enum
from bitarray import bitarray
import struct
import zlib

# Documentation can be found here: https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md

# The lenght in bytes of the full OOTX data frame. The OOTX data frame contains the cal info (among other things) sent by the base station
OOTX_MAX_FRAME_LENGTH = 43

def betole (value: bitarray) -> bitarray:
    first_eight = value[0:8]
    second_eight = value[8:16]
    return bitarray(second_eight + first_eight)

class RXState(Enum):
    """Defines what frame is currently being decoded, following the Light Emissions doc."""
    rxLength = 0
    rxData = 1
    rxCrc0 = 2
    rxCrc1 = 3
    rxDone = 4

@dataclass
class OOTXDataFrame:
    """The intrinsic base station calibration information."""
    protocolVersion: int = 0
    firmwareVersion: int = 0
    id: int = 0
    phase0: float = 0.0
    phase1: float = 0.0
    tilt0: float = 0.0
    tilt1: float = 0.0
    unlockCount: int = 0
    hwVersion: int = 0
    curve0: float = 0.0
    curve1: float = 0.0
    accelX: int = 0
    accelY: int = 0
    accelZ: int = 0
    gibphase0: float = 0.0
    gibphase1: float = 0.0
    gibmag0: float = 0.0
    gibmag1: float = 0.0
    mode: int = 0
    faults: int = 0
    ogeephase0: float = 0.0
    ogeephase1: float = 0.0
    ogeemag0: float = 0.0
    ogeemag1: float = 0.0
    DECODING_FORMAT: str = ">HHLeeeeBBeebbbeeeeBBeeee"

class OOTXDecoder:
    """Class in charge of receiving the bs calibration slow bits and decoding them."""
    def __init__(self, logger):
        self.is_fully_decoded: bool = False # Calibration is decoded and valid

        self.current_word: bitarray = bitarray()

        self.payload_length: int = 0  # Length of the payload in bytes

        self.crc32: int = 0         # Used to check integrity of data
        self.crc32_1: int = 0         # Used to check integrity of data

        self.n_zeros: int = 0       # Used to count 0s and look for the sync frame
        self.synced: bool = False

        self.data: list[bitarray] = []              # Raw data, 43 bytes
        self.frame: OOTXDataFrame = OOTXDataFrame() # Decoded data
        self.logger = logger

    def ootx_decoder_process_bit(self, data: int) -> bool:
        """
        Processes the calibration slow bit.

        Args:
            data (int): The base station calibration slow bit

        Returns:
            bool: If the calibration was fully or not
        """
        bit = data & 1

        # Start looking for the sync, 17 zeros followed by a one
        if not self.synced and not self.is_fully_decoded:
            if bit == 0:
                self.n_zeros += 1
            elif bit == 1 and self.n_zeros == 17:
                self.synced = True
                self.logger.info("Found cal sync, reading cal payload length")
            else:
                self.n_zeros = 0
            return

        # Start reading length. If length was retrieved, continue with data
        if not self.payload_length:
            self.current_word.append(bit)
            # Read 17 bits given there's one sync bit, discard it when unpacking
            if len(self.current_word) == 17:
                self.current_word.pop()
                self.payload_length = struct.unpack("<H", self.current_word.tobytes())[0]
                self.current_word.clear()
                self.logger.info("Found payload length: " + str(self.payload_length))
            return

        # Start reading data. If all data was retrieved, continue with CRC32
        if len(self.data) < self.payload_length:
            self.current_word.append(bit)
            # Read 17 bits given there's one sync bit, discard it when unpacking
            if len(self.current_word) == 17:
                self.current_word.pop()
                self.data.append(betole(self.current_word))
                self.current_word.clear()
                self.logger.info("Found " + str(len(self.data)) + " out of " + str(self.payload_length) + " payload bytes")
            return

        # Start reading CRC32. If it was retrieved, verify and finish
        if not self.crc32:
            self.current_word.append(bit)
            # Read 17 bits given there's one sync bit, discard it when unpacking
            if len(self.current_word) == 17 and not self.crc32_1:
                self.current_word.pop()
                self.crc32_1 = struct.unpack("<H", self.current_word.tobytes())[0]
                self.current_word.clear()
                self.logger.info("Found first half of the CRC32")
            elif self.crc32_1 and len(self.current_word) == 17:
                self.current_word.pop()
                self.crc32 = self.crc32_1 | (struct.unpack("<H", self.current_word.tobytes())[0] << 16)
                self.current_word.clear
                self.logger.info("Found second half of the CRC32")
            return

        # Check crc and extract data if good
        if self.check_crc():
            self.logger.info("Verified CRC32, fully decoded")
            self.is_fully_decoded = True
            self.extract_frame()
        else:
            self.logger.info("CRC32 verification failed")
            # Reset everything, something went wrong
            self.is_fully_decoded = False
            self.current_word.clear()
            self.payload_length = 0
            self.crc32 = 0
            self.crc32_1 = 0
            self.n_zeros = 0
            self.synced = False
            self.data.clear()

    def check_crc(self) -> bool:
        """Computes the CRC32 of the complete received cal data and compares against the received CRC32."""
        # received_crc32 = zlib.crc32(self.data)
        crc = 0
        # Calculate CRC32 for each bitarray element
        for ba in self.data:
            # Convert bitarray to bytes
            ba_bytes = ba.tobytes()
            # Update CRC32
            crc = zlib.crc32(ba_bytes, crc)

        self.logger.info("Calculated crc: " + str(crc) + " vs received crc: " + str(self.crc32))
        return crc == self.crc32

    def extract_frame(self):
        """Extracts the OOTX frame info from the saved data when decodification finished."""
        full_bitarray = bitarray()
        for arr in self.data:
            full_bitarray.extend(arr)
        try:
            self.frame = struct.unpack(OOTXDataFrame.DECODING_FORMAT, full_bitarray.tobytes())[0]
        except Exception as e:
            print(f"Failed to unpack cal frame: {e}")
