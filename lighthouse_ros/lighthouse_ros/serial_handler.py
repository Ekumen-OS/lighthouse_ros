from serial import Serial
from dataclasses import dataclass, field
import struct

from lighthouse_ros.pulse_processor import PulseProcessorFrame

# Length of the received uart frame from the base station
UART_FRAME_LENGTH = 12

@dataclass
class LighthouseUartFrame:
    data: PulseProcessorFrame = field(default_factory=lambda: PulseProcessorFrame())
    is_sync_frame: bool = False

class SerialHandler:
  """Class to handle serial reading and parsing."""
  def __init__(self, port: str, logger) -> None:
    self.logger = logger
    """Initializes the serial port."""
    if port.startswith("/dev/"):
        self.src = Serial(port, 2*115200)
    else:
        self.src = open(port, "rb")

  def wait_for_sync(self) -> None:
    """Waits for the sync frame coming from the deck."""
    self.logger.info("Waiting for sync ...")
    sync = False
    sync_counter = 0
    while not sync:
        reading = self.src.read(1)
        if reading == b'\xff':
            sync_counter += 1
        else:
            sync_counter = 0
        sync = (sync_counter == UART_FRAME_LENGTH)
    self.logger.info("Found sync!")

  def get_uart_frame_raw(self) -> tuple[bool, LighthouseUartFrame]:
    """Reads a frame from the serial port and parses it to fill a lighthouse UART frame."""
    reading = self.src.read(UART_FRAME_LENGTH)

    lighthouse_uart_frame = LighthouseUartFrame() # Reset data structure

    # Sync frame
    if reading == 0xffffffffffffffffffffffff:
        lighthouse_uart_frame.is_sync_frame = True
    else:
        lighthouse_uart_frame.is_sync_frame = False

    lighthouse_uart_frame.data.timestamp = struct.unpack("<I", reading[9:] + b'\x00')[0]
    lighthouse_uart_frame.data.beam_data = struct.unpack("<I", reading[6:9] + b'\x00')[0]
    offset_6 = struct.unpack("<I", reading[3:6] + b'\x00')[0]
    first_word = struct.unpack("<I", reading[:3] + b'\x00')[0]

    # Offset is expressed in a 6 MHz clock, while the timestamp uses a 24 MHz clock.
    # update offset to a 24 MHz clock
    lighthouse_uart_frame.data.offset = offset_6 * 4

    lighthouse_uart_frame.data.sensor = first_word & 0x03
    lighthouse_uart_frame.data.width = (first_word >> 8) & 0xffff

    identity = (first_word >> 2) & 0x1f
    lighthouse_uart_frame.data.channel = 0
    lighthouse_uart_frame.data.channel_found = True
    lighthouse_uart_frame.data.slow_bit = identity & 1

    is_padding_zero = (((reading[5] | reading[8]) & 0xfe) == 0)
    return (is_padding_zero or lighthouse_uart_frame.is_sync_frame, lighthouse_uart_frame)
