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


def get_uart_frame_raw(serial_port: Serial, ligthouse_uart_frame: LighthouseUartFrame) -> bool:
    reading = serial_port.read(UART_FRAME_LENGTH)

    ligthouse_uart_frame = LighthouseUartFrame() # Reset data structure

    is_padding_zero = False

    # Unpack
    ligthouse_uart_frame = LighthouseUartFrame()
    ligthouse_uart_frame.data.timestamp = struct.unpack("<I", reading[9:] + b'\x00')[0]
    ligthouse_uart_frame.data.beam_data = struct.unpack("<I", reading[6:9] + b'\x00')[0]
    offset_6 = struct.unpack("<I", reading[3:6] + b'\x00')[0]
    first_word = struct.unpack("<I", reading[:3] + b'\x00')[0]
    # Sync frame
    if offset_6 == 0xffffff:
        ligthouse_uart_frame.is_sync_frame = True

    # Offset is expressed in a 6 MHz clock, while the timestamp uses a 24 MHz clock.
    # update offset to a 24 MHz clock
    ligthouse_uart_frame.data.offset = offset_6 * 4

    ligthouse_uart_frame.data.sensor = first_word & 0x03
    ligthouse_uart_frame.data.width = (first_word >> 8) & 0xffff

    nPoly_ok = ((first_word >> 7) & 0x01) == 0
    if nPoly_ok:
        identity = (first_word >> 2) & 0x1f
        ligthouse_uart_frame.data.channel = identity >> 1
        ligthouse_uart_frame.data.slow_bit = identity & 1
    else:
        ligthouse_uart_frame.data.channel = None
        ligthouse_uart_frame.data.slow_bit = None

    # TODO: Double check if this is copy/pasteable
    is_padding_zero = (((reading[5] | reading[8]) & 0xfe) == 0)

    return is_padding_zero or ligthouse_uart_frame.is_sync_frame

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
        while(True):
            wait_for_sync(serial_port)
            previous_was_sync_frame = False

            # Start the data aquisition
            ligthouse_uart_frame = LighthouseUartFrame()
            while get_uart_frame_raw(serial_port, ligthouse_uart_frame):
                # Reset angles
                if ligthouse_uart_frame.is_sync_frame and previous_was_sync_frame:
                    self.pulse_processor.pulse_processor_clear()

                elif not ligthouse_uart_frame.is_sync_frame:
                    self.process_frames(ligthouse_uart_frame)

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
