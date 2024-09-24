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
    DECODING_FORMAT: str = ">hieeeebbeebbbeeeebb"

class OOTXDecoder:
    """Class in charge of receiving the bs calibration slow bits and decoding them."""
    def __init__(self):
        self.n_zeros: int = 0
        self.synchronized: bool = False
        self.is_fully_decoded: bool = False
        self.rx_state: RXState = RXState.rxLength
        self.current_word: bitarray = bitarray()
        self.frame_length: int = 0
        self.crc32: int = 0

        self.data: list[bitarray] = []
        self.frame: OOTXDataFrame = OOTXDataFrame()

    def ootx_decoder_process_bit(self, data: int) -> bool:
        """Processes the calibration slow bit.

        Args:
            data (int): The base station calibration slow bit

        Returns:
            bool: If the calibration was fully or not
        """
        data &= 1

        # Look for the sync frame, which is 17 zeros and a one
        if self.n_zeros == 17 and data == 1:
            self.synchronized = True    # If sync frame was found
            self.is_fully_decoded = False       # If all data frames where decoded
            # If sync found, start reading the first frame which is the payload length
            self.rx_state = RXState.rxLength    # Current frame being read
            return False
        if data == 0:
            self.n_zeros += 1
        else:
            self.n_zeros = 0

        if self.synchronized:
            if len(self.current_word) == 16:
                if data == 0:
                    self.synchronized = False
                    self.is_fully_decoded = False
                    return False
                if self.rx_state == RXState.rxDone:
                    is_data_valid = self.check_crc()
                    self.is_fully_decoded = is_data_valid
                    self.synchronized = False
                    self.extract_frame()
                    return is_data_valid
                else:
                    self.is_fully_decoded = False
                    return False
            self.current_word.append(data)
            if len(self.current_word) == 16:
                match self.rx_state:
                    case RXState.rxLength:
                        self.frame_length = struct.unpack("<h", self.current_word.tobytes())[0]
                        if self.frame_length > OOTX_MAX_FRAME_LENGTH:
                            self.synchronized = False
                            self.is_fully_decoded = False
                            return False
                        self.rx_state = RXState.rxData
                    case RXState.rxData:
                        self.data.append(betole(self.current_word))
                        if 2 * len(self.data) >= self.frame_length:
                            self.rx_state = RXState.rxCrc0
                    case RXState.rxCrc0:
                        self.crc32 = struct.unpack("<h", self.current_word.tobytes())[0]
                        self.rx_state = RXState.rxCrc1
                    case RXState.rxCrc1:
                        self.crc32 |= (struct.unpack("<h", self.current_word.tobytes())[0] << 16)
                        self.rx_state = RXState.rxDone
                    case RXState.rxDone:
                        pass
                self.current_word.clear() # Clear current word and get ready for new one
        self.is_fully_decoded = False
        return False

    def check_crc(self) -> bool:
        """Computes the CRC32 of the complete received cal data and compares against the received CRC32."""
        received_crc32 = zlib.crc32(self.data)
        return received_crc32 == self.crc32

    def extract_frame(self):
        """Extracts the OOTX frame info from the saved data when decodification finished."""
        full_bitarray = bitarray()
        for arr in self.data:
            full_bitarray.extend(arr)
        try:
            self.frame = struct.unpack(OOTXDataFrame.DECODING_FORMAT, full_bitarray.tobytes())[0]
        except Exception as e:
            print(f"Failed to unpack cal frame: {e}")
