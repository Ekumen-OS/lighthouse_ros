import sys, struct
import config
import time
from dataclasses import dataclass
from pulse_processor import PulseProcessor, PulseProcessorFrame
from ootx_decoder import OOTXDecoder
from lighthouse_calibration import LighthouseCalibration
from smbus2 import SMBus
from serial import Serial

# Hand testing:
# Create a virtual serial in one terminal: socat -d -d pty,raw,echo=0 pty,raw,echo=0
# Run the core.py script passing the virtual serial port
# Send byte words with: printf '%b' 'bytes' > /dev/pts/...

# Length of the received uart frame from the base station
UART_FRAME_LENGTH = 12

@dataclass
class LighthouseUartFrame:
    data: PulseProcessorFrame
    is_sync_frame: bool

    def __init__(self):
        self.data = PulseProcessorFrame()
        self.is_sync_frame = False


def get_uart_frame_raw(serial_port: Serial, lighthouse_uart_frame: LighthouseUartFrame) -> bool:
    reading = serial_port.read(UART_FRAME_LENGTH)

    lighthouse_uart_frame = LighthouseUartFrame() # Reset data structure

    # Sync frame
    if reading == 0xffffffffffffffffffffffff:
        lighthouse_uart_frame.is_sync_frame = True
    else:
        lighthouse_uart_frame.is_sync_frame = False

    # Unpack
    lighthouse_uart_frame = LighthouseUartFrame()

    lighthouse_uart_frame.data.sensor = reading[0] & 0x03
    lighthouse_uart_frame.data.channel_found = (reading[0] & 0x80) == 0
    # lighthouse_uart_frame.data.channel = (reading[0] >> 3) & 0x0f
    lighthouse_uart_frame.data.channel = 0  # Hardcode to channel 0 in our case
    lighthouse_uart_frame.data.slow_bit = (reading[0] >> 2) & 0x01
    lighthouse_uart_frame.data.width = reading[1:2]
    lighthouse_uart_frame.data.offset = reading[3:6]
    lighthouse_uart_frame.data.beam_data = reading[6:9]
    lighthouse_uart_frame.data.timestamp = reading[9:11]

    # Offset is expressed in a 6 MHz clock, while the timestamp uses a 24 MHz clock.
    # update offset to a 24 MHz clock
    lighthouse_uart_frame.data.offset *= 4

    is_padding_zero = False
    is_padding_zero = (((reading[5] | reading[8]) & 0xfe) == 0)

    return is_padding_zero or lighthouse_uart_frame.is_sync_frame

def wait_for_sync(src: Serial):
    # Wait for sync
    print("Waiting for sync ...")
    sync = [b'\xff', b'\xff', b'\xff', b'\xff', b'\xff', b'\xff', b'\xff', b'\xff', b'\xff', b'\xff', b'\xff', b'\xff']
    syncBuffer = [b'\x00'] * len(sync)
    while sync != syncBuffer:
        b = src.read(1)
        if len(b) < 1:
            sys.exit(1)
        syncBuffer.append(b)
        syncBuffer = syncBuffer[1:]

    print("Found sync!")

class LighthouseCore:
    def __init__(self, serial_port: Serial):
        # Class variables
        self.ootx_decoder = [OOTXDecoder()] * config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS
        self.pulse_processor = PulseProcessor(self.ootx_decoder)

        i2c_address = 0x2f
        bus = SMBus(1)
        time.sleep(1)
        # Write a 0 to get out of the bootloader mode and start receiving data via SPI
        var = bus.write_byte_data(i2c_address, 0, 0)

        # Start the infinite position estimation loop
        self.core_task(serial_port)

    def core_task(self, serial_port: Serial):
        # TODO: This is inside the "while true" in the firmware, but sync frames are sent only once for v2 so it doesn't make sense?
        wait_for_sync(serial_port)

        while(True):
            previous_was_sync_frame = False

            # Start the data aquisition
            lighthouse_uart_frame = LighthouseUartFrame()
            while get_uart_frame_raw(serial_port, lighthouse_uart_frame):
                # Reset angles
                if lighthouse_uart_frame.is_sync_frame and previous_was_sync_frame:
                    print ("clear")
                    self.pulse_processor.pulse_processor_clear()

                elif not lighthouse_uart_frame.is_sync_frame:
                    print("process")
                    self.process_frames(lighthouse_uart_frame)

                previous_was_sync_frame = lighthouse_uart_frame.is_sync_frame

    def process_frames(self, frame: LighthouseUartFrame):
        base_station = int()
        sweep_id = int()

        (result, base_station, sweep_id, calib_data_is_decoded) = self.pulse_processor.process_pulse(frame.data)
        if result:
            self.use_pulse_result(base_station, sweep_id)

        if calib_data_is_decoded:
            self.use_calibration_data()

    def use_pulse_result(self):
        # TODO
        print ("Using pulse result... Code it in")

    def use_calibration_data(self):
        for i in range(config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS):
            if self.ootx_decoder[i].is_fully_decoded():
                new_cal_data = LighthouseCalibration()
                new_cal_data = self.lighthouse_calibration_init_from_frame(i)
                if (new_cal_data.uid != self.pulse_processor.base_station_calibration[i].uid) or (new_cal_data.valid != self.pulse_processor.base_station_calibration[i].valid):
                    # Received new cal, update
                    self.pulse_processor.base_station_calibration[i] = new_cal_data
                    # This is where we would save to "disk" if we want to

    def lighthouse_calibration_init_from_frame(self, i: int) -> LighthouseCalibration:
        new_cal_data = LighthouseCalibration()
        new_cal_data.sweep[0].phase = self.ootx_decoder[i].frame.phase0
        new_cal_data.sweep[0].tilt = self.ootx_decoder[i].frame.tilt0
        new_cal_data.sweep[0].curve = self.ootx_decoder[i].frame.curve0
        new_cal_data.sweep[0].gibmag = self.ootx_decoder[i].frame.gibmag0
        new_cal_data.sweep[0].gibphase = self.ootx_decoder[i].frame.gibphase0
        new_cal_data.sweep[0].ogeemag = self.ootx_decoder[i].frame.ogeemag0
        new_cal_data.sweep[0].ogeephase = self.ootx_decoder[i].frame.ogeephase0

        new_cal_data.sweep[1].phase = self.ootx_decoder[i].frame.phase1
        new_cal_data.sweep[1].tilt = self.ootx_decoder[i].frame.tilt1
        new_cal_data.sweep[1].curve = self.ootx_decoder[i].frame.curve1
        new_cal_data.sweep[1].gibmag = self.ootx_decoder[i].frame.gibmag1
        new_cal_data.sweep[1].gibphase = self.ootx_decoder[i].frame.gibphase1
        new_cal_data.sweep[1].ogeemag = self.ootx_decoder[i].frame.ogeemag1
        new_cal_data.sweep[1].ogeephase = self.ootx_decoder[i].frame.ogeephase1

        new_cal_data.uid = self.ootx_decoder[i].frame.id
        new_cal_data.valid = True
        return new_cal_data

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: {} <input.bin or /dev/tty...>".format(sys.argv[0]))
        exit(1)
    if sys.argv[1].startswith("/dev/"):
        src = Serial(sys.argv[1], 2*115200)
    else:
        src = open(sys.argv[1], "rb")

    lighthouse_core = LighthouseCore(src)
