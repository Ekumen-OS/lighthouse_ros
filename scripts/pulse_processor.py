from dataclasses import dataclass
from lighthouse_calibration import LighthouseCalibration, LighthouseCalibrationSweep
import config
import time
import math

# Base stations perform an horizontal and a vertical sweep
PULSE_PROCESSOR_N_SWEEPS = 2

# 4 sensors in the PCB
PULSE_PROCESSOR_N_SENSORS = 4

NO_OFFSET = 0
PULSE_PROCESSOR_TIMESTAMP_BITWIDTH = 24
PULSE_PROCESSOR_TIMESTAMP_MAX = ((1 << PULSE_PROCESSOR_TIMESTAMP_BITWIDTH) - 1)
PULSE_PROCESSOR_TIMESTAMP_BITMASK = PULSE_PROCESSOR_TIMESTAMP_MAX
MIN_TICKS_BETWEEN_SLOW_BITS = int((887000 / 2) * 8 / 10)
MAX_TICKS_BETWEEN_SWEEP_STARTS_TWO_BLOCKS = int(10)
MAX_TICKS_SENSOR_TO_SENSOR = int(10000)

V2_N_CHANNELS = 16
CYCLE_PERIODS = [ 959000 / 2, 957000 / 2, 953000 / 2, 949000 / 2, 947000 / 2, 943000 / 2, 941000 / 2, 939000 / 2, 937000 / 2, 929000 / 2, 919000 / 2, 911000 / 2, 907000 / 2, 901000 / 2, 893000 / 2, 887000 / 2]

PULSE_PROCESSOR_N_CONCURRENT_BLOCKS = 2
PULSE_PROCESSOR_N_WORKSPACE = (PULSE_PROCESSOR_N_SENSORS * PULSE_PROCESSOR_N_CONCURRENT_BLOCKS)

def cycle_period_to_microseconds(cyclePeriod):
    return cyclePeriod / 24

def ts_diff(x, y) -> int:
  return int((x - y)) & PULSE_PROCESSOR_TIMESTAMP_BITMASK

def ts_abs_diff_larger_than(a, b, limit) -> int:
    return ts_diff(a + limit, b) > (limit * 2)

@dataclass
class PulseProcessorFrame:
    # Structure definition at https://github.com/bitcraze/lighthouse-fpga
    is_sync_frame: bool
    sensor: int
    timestamp: int
    width: int
    beam_data: int
    offset: int
    channel: int
    slow_bit: int
    channel_found: bool

    def __init__(self):
        self.is_sync_frame = False
        self.sensor = 0
        self.timestamp = 0
        self.width = 0
        self.beam_data = 0
        self.offset = 0
        self.channel = 0
        self.slow_bit = 0
        self.channel_found = False

@dataclass
class PulseProcessorSensorMeasurement:
    # Angles and corrected angles have the same length, both PULSE_PROCESSOR_N_SWEEPS==2
    angles: list[float]
    corrected_angles: list[float]
    valid_count: int

    def __init__(self):
        self.angles = [0.0] * PULSE_PROCESSOR_N_SWEEPS
        self.corrected_angles = [0.0] * PULSE_PROCESSOR_N_SWEEPS

@dataclass
class PulseProcessorBaseStationMeasurement:
    sensor_measurements: list[PulseProcessorSensorMeasurement]

    def __init__(self):
        self.sensor_measurements = [PulseProcessorSensorMeasurement()] * PULSE_PROCESSOR_N_SENSORS

@dataclass
class PulseProcessorResult:
    base_station_measurements: list[PulseProcessorBaseStationMeasurement]
    last_usec_timestamp: list[int]

    def __init__(self):
        self.base_station_measurements = [PulseProcessorBaseStationMeasurement()] * config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS
        self.last_usec_timestamp = [0] * config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS

@dataclass
class PulseProcessorPulseWorkspace:
    slots_used: int
    latest_timestamp: int
    slots: list[PulseProcessorFrame]

    def __init__(self):
        self.slots_used = 0
        self.latest_timestamp = 0
        self.slots = [PulseProcessorFrame()] * PULSE_PROCESSOR_N_WORKSPACE

@dataclass
class PulseProcessorSweepBlock:
    offset: list[int]
    timestamp: int
    channel: int

    def __init__(self):
        self.offset = [0] * PULSE_PROCESSOR_N_SENSORS
        self.timestamp = 0
        self.channel = 0

@dataclass
class PulseProcessorBlockWorkspace:
    blocks: list[PulseProcessorSweepBlock]

    def __init__(self):
        self.blocks = [PulseProcessorSweepBlock()] * PULSE_PROCESSOR_N_CONCURRENT_BLOCKS

class PulseProcessor:
    def __init__(self, ootx_decoder):
        self.ootx_decoder = ootx_decoder
        self.base_station_calibration = [LighthouseCalibration()] * config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS
        self.angles = PulseProcessorResult()

        # pulseProcessorV2 struct
        self.pulse_workspace = PulseProcessorPulseWorkspace()
        self.block_workspace = PulseProcessorBlockWorkspace()
        self.blocks = PulseProcessorSweepBlock()
        self.ootx_timestamps = [0] * config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS

    def process_pulse(self, frame_data: PulseProcessorFrame):
        # TODO: Momentarily disabled, will handle calib later
        # calib_data_is_decoded = self.handle_calibration_data(frame_data)
        calib_data_is_decoded = False
        result, base_station, sweep_id = self.handle_angles(frame_data)
        return result, base_station, sweep_id, calib_data_is_decoded

    def handle_calibration_data(self, frame_data: PulseProcessorFrame) -> bool:
        is_full_message = False

        if frame_data.channel_found and frame_data.channel < config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS:
            if frame_data.offset != NO_OFFSET:
                prev_timestamp0 = self.ootx_timestamps[frame_data.channel]
                timestamp0 = ts_diff(frame_data.timestamp, frame_data.offset)
                if ts_abs_diff_larger_than(timestamp0, prev_timestamp0, MIN_TICKS_BETWEEN_SLOW_BITS):
                    is_full_message = self.ootx_decoder[frame_data.channel].ootx_decoder_process_bit(frame_data.slow_bit)
                self.ootx_timestamps[frame_data.channel] = timestamp0
        return is_full_message

    def handle_angles(self, frame_data: PulseProcessorFrame):
        # TODO: Disabled, will clear angles after printing them in core
        # self.clear_stale_angles_after_timeout()

        n_of_blocks = self.process_frame(frame_data)
        for i in range(n_of_blocks):
            block = self.block_workspace.blocks[i]
            channel = block.channel
            if channel < config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS:
                previous_block = self.block_workspace.blocks[channel]
                if self.is_block_pair_good(block, previous_block):
                    self.calculate_angles(block, previous_block)
                    return True, channel, 1
                else:
                    self.blocks[channel] = self.block_workspace.blocks[i]
        return False, 0, 0

    def is_block_pair_good(self, latest: PulseProcessorSweepBlock, storage: PulseProcessorSweepBlock) -> bool:
        if latest.channel != storage.channel:
            return False
        if ts_abs_diff_larger_than(latest.timestamp, storage.timestamp, MAX_TICKS_BETWEEN_SWEEP_STARTS_TWO_BLOCKS):
            return False
        return True

    def calculate_angles(self, latest: PulseProcessorSweepBlock, previous: PulseProcessorSweepBlock):
        channel = latest.channel

        for i in range(PULSE_PROCESSOR_N_SENSORS):
            first_offset = previous.offset[i]
            second_offset = latest.offset[i]
            period = CYCLE_PERIODS[channel]

            first_beam = (first_offset * 2 * math.pi / period) - math.pi + math.pi / 3
            second_beam = (second_offset * 2 * math.pi / period) - math.pi - math.pi / 3

            self.angles.base_station_measurements[channel].sensor_measurements[i].angles[0] = first_beam
            self.angles.base_station_measurements[channel].sensor_measurements[i].angles[1] = second_beam
            self.angles.base_station_measurements[channel].sensor_measurements[i].valid_count = 2
        self.angles.last_usec_timestamp[channel] = round(time.time() * 1000)  # TODO: This should be usec

    def clear_stale_angles_after_timeout(self):
        current_time_us = round(time.time() * 1000) # TODO: This should be usec
        for i in range(config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS):
            elapsed_time_ms = current_time_us - self.angles.last_usec_timestamp[i]
            if elapsed_time_ms > cycle_period_to_microseconds(CYCLE_PERIODS[i]):
                self.pulse_processor_clear(i)

    def clear_stale_angles(self):
        for i in range(config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS):
            self.pulse_processor_clear(i)

    def pulse_processor_clear(self, i):
        for j in range(PULSE_PROCESSOR_N_SENSORS):
            self.angles.base_station_measurements[i].sensor_measurements[j].valid_count = 0

    def process_frame(self, frame_data: PulseProcessorFrame):
        n_of_blocks = 0

        is_first_frame_in_new_workspace = ts_abs_diff_larger_than(frame_data.timestamp, self.pulse_workspace.latest_timestamp, MAX_TICKS_SENSOR_TO_SENSOR)
        if is_first_frame_in_new_workspace:
            n_of_blocks = self.process_workspace()
            self.clear_workspace()

        # Not needed with the new approach
        self.pulse_workspace.latest_timestamp = frame_data.timestamp

        if not self.store_pulse(frame_data):
            self.clear_workspace()
        return n_of_blocks

    def clear_workspace(self):
        self.pulse_workspace.slots_used = 0

    def store_pulse(self, frame_data: PulseProcessorFrame):
        if self.pulse_workspace.slots_used < PULSE_PROCESSOR_N_WORKSPACE:
            self.pulse_workspace.slots[self.pulse_workspace.slots_used] = frame_data
            self.pulse_workspace.slots_used += 1
            return True
        return False

    def process_workspace(self) -> int:
        # In case a frame or frames in the workspace did not arrive with a channel (basestation ID),
        # look for the frame that does have a channel and assign that to the other frames in the block workspace.
        # We tecnically do not need this as we hardcode all channels to 0.
        self.augment_frames_in_workspace()

        slots_used = self.pulse_workspace.slots_used
        if slots_used < PULSE_PROCESSOR_N_SENSORS:
            return 0
        if (slots_used % PULSE_PROCESSOR_N_SENSORS) != 0:
            return 0
        blocks_in_workspace = int(slots_used / PULSE_PROCESSOR_N_SENSORS)
        for block_index in range(blocks_in_workspace):
            block_base_index = block_index * PULSE_PROCESSOR_N_SENSORS
            if not self.process_workspace_block(block_base_index, block_index):
                return 0
        return blocks_in_workspace

    def augment_frames_in_workspace(self):
        slots_used = self.pulse_workspace.slots_used
        channel_is_known = False
        channel = 0

        for i in range(slots_used - 1, -1, -1):
            if self.pulse_workspace.slots[i].channel_found:
                channel = self.pulse_workspace.slots[i].channel
                channel_is_known = True
            else:
                if (channel_is_known):
                    self.pulse_workspace.slots[i].channel = channel
                    self.pulse_workspace.slots[i].channel_found = True

    def process_workspace_block(self, block_base_index: int, block_index: int) -> bool:
        sensor_mask = 0
        NO_SENSOR = -1
        NO_OFFSET = 0
        NO_CHANNEL = 0xff

        # Check there's data for all sensors
        for i in range(PULSE_PROCESSOR_N_SENSORS):
            sensor_mask |= (1 << self.pulse_workspace.slots[block_base_index + i].sensor)
        if sensor_mask != 0xf:
            return False

        # Check channel is the same for all frames
        self.block_workspace.blocks[block_index].channel = NO_CHANNEL # No channel
        for i in range(PULSE_PROCESSOR_N_SENSORS):
            if self.pulse_workspace.slots[block_base_index + i].channel_found:
                if self.block_workspace.blocks[block_index].channel == NO_CHANNEL:
                    self.block_workspace.blocks[block_index].channel = self.pulse_workspace.slots[block_base_index + i].channel
                if self.block_workspace.blocks[block_index].channel != self.pulse_workspace.slots[block_base_index + i].channel:
                    return False
        if self.block_workspace.blocks[block_index].channel == NO_CHANNEL:
            return False

        # Only one sensor should have offset?? Not happening in testing
        index_with_offset = NO_SENSOR
        for i in range(PULSE_PROCESSOR_N_SENSORS):
            if self.pulse_workspace.slots[block_base_index + i].offset != NO_OFFSET:
                if index_with_offset == NO_SENSOR:
                    index_with_offset = i
                else:
                    return False

        if index_with_offset == NO_SENSOR:
            return False

        # Calculate offsets for all sensors
        for i in range(PULSE_PROCESSOR_N_SENSORS):
            sensor = self.pulse_workspace.slots[block_base_index + i].sensor
            if i == index_with_offset:
                self.block_workspace.blocks[block_index].offset[sensor] = self.pulse_workspace.slots[block_base_index + i].sensor
            else:
                timestamp_delta = ts_diff(self.pulse_workspace.slots[block_base_index + index_with_offset].timestamp, self.pulse_workspace.slots[block_base_index + i].timestamp)
                self.block_workspace.blocks[block_index].offset[sensor] = ts_diff(self.pulse_workspace.slots[block_base_index + index_with_offset].offset, timestamp_delta)

        self.block_workspace.blocks[block_index].timestamp = ts_diff(self.pulse_workspace.slots[block_base_index + index_with_offset].timestamp, self.pulse_workspace.slots[block_base_index + index_with_offset].offset)

        return True

    def apply_calibration(self, bs: int):
        """Applies the bs calibration to the measured angles before estimating pos."""
        if self.base_station_calibration[bs].valid:
            max_delta = 0.0005
            for sensor in range(PULSE_PROCESSOR_N_SENSORS):
                self.angles.base_station_measurements[bs].sensor_measurements[sensor].corrected_angles = self.angles.base_station_measurements[bs].sensor_measurements[sensor].angles

                # Don't know why 5 times
                for i in range(5):
                    current_distorted_angles = self.ideal_to_distorted(self.angles.base_station_measurements[bs].sensor_measurements[sensor].corrected_angles, self.base_station_calibration[bs])
                    delta0 = self.angles.base_station_measurements[bs].sensor_measurements[sensor].angles[0] - current_distorted_angles[0]
                    delta1 = self.angles.base_station_measurements[bs].sensor_measurements[sensor].angles[1] - current_distorted_angles[1]

                    self.angles.base_station_measurements[bs].sensor_measurements[sensor].corrected_angles[0] += delta0
                    self.angles.base_station_measurements[bs].sensor_measurements[sensor].corrected_angles[1] += delta1

                    if abs(delta0) < max_delta and abs(delta1) < max_delta:
                        break

    def ideal_to_distorted(self, ideal: list, calib: LighthouseCalibrationSweep) -> list:
        t30 = math.pi / 6
        tan30 = 0.5773502691896258

        a1 = ideal[0]
        a2 = ideal[1]

        x = 1.0
        y = math.tan((a2 + a1) / 2.0)
        z = math.sin((a2 - a1) / (tan30 * (math.cos(a2) * math.cos(a1))))

        return [self.apply_lh2_model(x, y, z, -t30, calib[0]), self.apply_lh2_model(x, y, z, t30, calib[1])]

    def apply_lh2_model(self, x: float, y: float, z: float, t: float, calib: LighthouseCalibrationSweep) -> float:
        ax = math.atan2(y, x)
        r = math.sqrt(x * x + y * y)

        to_clip = z * math.tan(t - calib.tilt) / r
        if to_clip < -1.0:
            to_clip = -1.0
        if to_clip > 1.0:
            to_clip = 1.0

        base = ax + math.asin(to_clip)
        comp_gib = -calib.gibmag * math.cos(ax + calib.gibphase)
        return base - (calib.phase + comp_gib)
