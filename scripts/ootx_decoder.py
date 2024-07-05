from dataclasses import dataclass
from enum import Enum

# WARN: This module might be a cause of failures. Lots of byte ops with uints, ulongs, etc.


OOTX_MAX_FRAME_LENGTH = 43

class RXState(Enum):
    rxLenght = 1
    rxData = 2
    rxCrc0 = 3
    rxCrc1 = 4
    rxDone = 5

def betole (value):
    return ((value & 0xff00) >> 8) | ((value & 0xff) << 8)

@dataclass
class OOTXDataFrame:
    protocolVersion: int
    firmwareVersion: int
    id: int
    phase0: float
    phase1: float
    tilt0: float
    tilt1: float
    unlockCount: int
    hwVersion: int
    curve0: float
    curve1: float
    accelX: int
    accelY: int
    accelZ: int
    gibphase0: float
    gibphase1: float
    gibmag0: float
    gibmag1: float
    mode: int
    faults: int
    ogeephase0: float
    ogeephase1: float
    ogeemag0: float
    ogeemag1: float

class OOTXDecoder:
    def __init__(self):
        self.n_zeros: int
        self.synchronized: bool
        self.bit_in_word: int
        self.word_received: int
        self.is_fully_decoded: bool
        self.rx_state: RXState
        self.current_word: int
        self.frame_lenght: int
        self.data: list[int] = [0] * int(((OOTX_MAX_FRAME_LENGTH + 1) / 2))
        self.frame: OOTXDataFrame
        self.crc32: int

    def ootx_decoder_process_bit(self, data) -> bool:
        data &= 1

        if self.n_zeros == 17 and data == 1:
            self.synchronized = True
            self.bit_in_word = 0
            self.word_received = 0
            self.is_fully_decoded = False
            self.rx_state = RXState.rxLength
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
                    is_data_valid = self.check_crc()        # TODO
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
                    case RXState.rxLenght:
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
