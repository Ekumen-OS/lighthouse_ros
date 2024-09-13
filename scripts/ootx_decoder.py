from dataclasses import dataclass
from enum import Enum
import zlib

# Documentation can be found here: https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md

# The lenght in bytes of the full OOTX data frame. The OOTX data frame contains the cal info (among other things) sent by the base station
OOTX_MAX_FRAME_LENGTH = 43

def betole (value: int) -> int:
    return ((value & 0xff00) >> 8) | ((value & 0xff) << 8)

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

class OOTXDecoder:
    """Class in charge of receiving the bs calibration slow bits and decoding them."""
    def __init__(self):
        self.n_zeros: int = 0
        self.synchronized: bool = False
        self.bit_in_word: int = 0
        self.word_received: int = 0
        self.is_fully_decoded: bool = False
        self.rx_state: RXState = RXState.rxLength
        self.current_word: int = 0
        self.frame_lenght: int = 0
        self.crc32: int = 0

        # TODO: The C firmware uses a union for these two data and frame values. Cannot replicate this in Python
        self.data: list[int] = [0] * int(((OOTX_MAX_FRAME_LENGTH + 1) / 2))
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
            self.bit_in_word = 0        # What bit number of the current word we are reading
            self.word_received = 0      # What byte in the data we are currently filling
            self.is_fully_decoded = False       # If all data frames where decoded
            # If sync found, start reading the first frame which is the payload length
            self.rx_state = RXState.rxLength    # Current frame being read
            return False
        if data == 0:
            self.n_zeros += 1
        else:
            self.n_zeros = 0

        if self.synchronized:
            if self.bit_in_word == 16:
                if data == 0:
                    self.synchronized = False
                    self.is_fully_decoded = False
                    return False
                self.bit_in_word = 0
                if self.rx_state == RXState.rxDone:
                    is_data_valid = self.check_crc()
                    self.is_fully_decoded = is_data_valid
                    self.synchronized = False
                    return is_data_valid
                else:
                    self.is_fully_decoded = False
                    return False
            self.current_word = (self.current_word<<1) | data
            self.bit_in_word += 1
            if self.bit_in_word == 16:
                match self.rx_state:
                    case RXState.rxLength:
                        self.frame_length = betole(self.current_word)
                        if self.frame_lenght > OOTX_MAX_FRAME_LENGTH:
                            self.synchronized = False
                            self.is_fully_decoded = False
                            return False
                        self.rx_state = RXState.rxData
                    case RXState.rxData:
                        self.data[self.word_received] = betole(self.current_word)
                        self.word_received += 1
                        if 2 * self.word_received >= self.frame_lenght:
                            self.rx_state = RXState.rxCrc0
                    case RXState.rxCrc0:
                        self.crc32 = betole(self.current_word)
                        self.rx_state = RXState.rxCrc1
                    case RXState.rxCrc1:
                        self.crc32 |= (betole(self.current_word) << 16)
                        self.rx_state = RXState.rxDone
                    case RXState.rxDone:
                        pass
        self.is_fully_decoded = False
        return False

    def check_crc(self) -> bool:
        """Computes the CRC32 of the complete received cal data and compares against the received CRC32."""
        received_crc32 = zlib.crc32(self.data)
        return received_crc32 == self.crc32
