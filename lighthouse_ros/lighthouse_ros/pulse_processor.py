from dataclasses import dataclass
import time
import math

import lighthouse_ros.config as config
from lighthouse_ros.math_helper import ts_abs_diff_larger_than, ts_diff

# Base stations perform a horizontal and a vertical sweep
PULSE_PROCESSOR_N_SWEEPS = 2

# 4 sensors in the PCB
PULSE_PROCESSOR_N_SENSORS = 4

NO_OFFSET = 0
MAX_TICKS_BETWEEN_SWEEP_STARTS_TWO_BLOCKS = int(10)
MAX_TICKS_SENSOR_TO_SENSOR = int(10000)

CYCLE_PERIODS = [ 959000 / 2, 957000 / 2, 953000 / 2, 949000 / 2, 947000 / 2, 943000 / 2, 941000 / 2, 939000 / 2, 937000 / 2, 929000 / 2, 919000 / 2, 911000 / 2, 907000 / 2, 901000 / 2, 893000 / 2, 887000 / 2]

PULSE_PROCESSOR_N_CONCURRENT_BLOCKS = 2
PULSE_PROCESSOR_N_WORKSPACE = (PULSE_PROCESSOR_N_SENSORS * PULSE_PROCESSOR_N_CONCURRENT_BLOCKS)

def cycle_period_to_microseconds(cyclePeriod):
    return cyclePeriod / 24

@dataclass
class PulseProcessorFrame:
    """A received frame from the lighthouse deck. Structure definition at https://github.com/bitcraze/lighthouse-fpga."""
    sensor: int
    timestamp: int
    width: int
    beam_data: int
    offset: int
    channel: int
    slow_bit: int
    channel_found: bool

    def __init__(self):
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
    """Main processing class that takes the UART frames sent by the deck and calculates the deck angles."""
    def __init__(self):
        self.angles = PulseProcessorResult() # Main result
        self.received_bs_sweep = [bool] * config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS

        # Structures used for the calculations
        self.pulse_workspace = PulseProcessorPulseWorkspace()
        self.block_workspace = PulseProcessorBlockWorkspace()
        self.blocks = [PulseProcessorSweepBlock()] * config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS

    def process_pulse(self, frame_data: PulseProcessorFrame) -> tuple[bool, int, int]:
        """Main entrypoint for the pulse processor."""
        angles_measured = False
        base_station = 0
        sweep_id = 0
        n_of_blocks = self.__process_frame(frame_data)
        for i in range(n_of_blocks):
            block = self.block_workspace.blocks[i]
            channel = block.channel
            if channel < config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS:
                if self.__is_block_pair_good(block, self.blocks[channel]):
                    self.__calculate_angles(block, self.blocks[channel])
                    angles_measured = True
                    base_station = channel
                    sweep_id = 1
                else:
                    self.blocks[channel] = self.block_workspace.blocks[i]
        return angles_measured, base_station, sweep_id

    def __is_block_pair_good(self, latest: PulseProcessorSweepBlock, storage: PulseProcessorSweepBlock) -> bool:
        """Checks if the sweeps belong to the same channel and are not far enough timewise."""
        if latest.channel != storage.channel:
            return False
        if ts_abs_diff_larger_than(latest.timestamp, storage.timestamp, MAX_TICKS_BETWEEN_SWEEP_STARTS_TWO_BLOCKS):
            return False
        return True

    def __calculate_angles(self, latest: PulseProcessorSweepBlock, previous: PulseProcessorSweepBlock) -> None:
        """Calculates the angles from the sweep blocks."""
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

    def clear_stale_angles(self):
        for i in range(config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS):
            self.pulse_processor_clear(i)

    def clear_outdated(self, bs: int):
        if self.received_bs_sweep[bs]:
            for i in range(config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS):
                if not self.received_bs_sweep[i]:
                    self.pulse_processor_clear(i)
                self.received_bs_sweep[i] = False
        self.received_bs_sweep[bs] = True

    def pulse_processor_clear(self, i):
        for j in range(PULSE_PROCESSOR_N_SENSORS):
            self.angles.base_station_measurements[i].sensor_measurements[j].valid_count = 0

    def __process_frame(self, frame_data: PulseProcessorFrame):
        n_of_blocks = 0

        is_first_frame_in_new_workspace = ts_abs_diff_larger_than(frame_data.timestamp, self.pulse_workspace.latest_timestamp, MAX_TICKS_SENSOR_TO_SENSOR)
        if is_first_frame_in_new_workspace:
            n_of_blocks = self.__process_workspace()
            self.__clear_workspace()

        # Not needed with the new approach
        self.pulse_workspace.latest_timestamp = frame_data.timestamp

        if not self.__store_pulse(frame_data):
            self.__clear_workspace()
        return n_of_blocks

    def __clear_workspace(self):
        self.pulse_workspace.slots_used = 0

    def __store_pulse(self, frame_data: PulseProcessorFrame):
        if self.pulse_workspace.slots_used < PULSE_PROCESSOR_N_WORKSPACE:
            self.pulse_workspace.slots[self.pulse_workspace.slots_used] = frame_data
            self.pulse_workspace.slots_used += 1
            return True
        return False

    def __process_workspace(self) -> int:
        # In case a frame or frames in the workspace did not arrive with a channel (basestation ID),
        # look for the frame that does have a channel and assign that to the other frames in the block workspace.
        # We tecnically do not need this as we hardcode all channels to 0.
        self.__augment_frames_in_workspace()

        slots_used = self.pulse_workspace.slots_used
        if slots_used < PULSE_PROCESSOR_N_SENSORS:
            return 0
        if (slots_used % PULSE_PROCESSOR_N_SENSORS) != 0:
            return 0
        blocks_in_workspace = int(slots_used / PULSE_PROCESSOR_N_SENSORS)
        for block_index in range(blocks_in_workspace):
            block_base_index = block_index * PULSE_PROCESSOR_N_SENSORS
            if not self.__process_workspace_block(block_base_index, block_index):
                return 0
        return blocks_in_workspace

    def __augment_frames_in_workspace(self):
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

    def __process_workspace_block(self, block_base_index: int, block_index: int) -> bool:
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

        # Check one sensor only has offset
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
