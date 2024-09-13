from dataclasses import dataclass

import math_helper
import config
from ootx_decoder import OOTXDecoder
from pulse_processor import PulseProcessorFrame

MIN_TICKS_BETWEEN_SLOW_BITS = int((887000 / 2) * 8 / 10)

@dataclass
class LighthouseCalibrationSweep:
    """Struct containing the calibration values."""
    phase: float
    tilt: float
    curve: float
    gibmag: float
    gibphase: float
    ogeemag: float
    ogeephase: float

    def __init__(self):
        self.phase = 0.0
        self.tilt = 0.0
        self.curve = 0.0
        self.gibmag = 0.0
        self.giphase = 0.0
        self.ogeemag = 0.0
        self. ogeephase = 0.0

@dataclass
class LighthouseCalibration:
    """Struct containing the intrinsic calibration for a base station."""
    uid: int        # The base station ID
    valid: bool     # If the stored calibration is valid or not
    sweep: list[LighthouseCalibrationSweep] # The actual calibration data

    def __init__(self):
        self.uid = 0
        self.valid = False
        self.sweep = [LighthouseCalibrationSweep()] * 2

class LighthouseCalibrator:
    """Class in charge of handling base station intrinsic calibrations."""
    def __init__(self) -> None:
        # The calibration for all base stations
        self.base_station_calibrations = [LighthouseCalibration()] * config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS
        # Object to read and decode the calibration from a UART frame
        self.ootx_decoder = [OOTXDecoder()] * config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS
        self.ootx_timestamps = [0] * config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS

    def handle_calibration_data(self, frame_data: PulseProcessorFrame):
        """Parses and saves the calibration data."""
        is_full_message = False

        if frame_data.channel_found and frame_data.channel < config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS:
            if frame_data.offset != 0:
                prev_timestamp0 = self.ootx_timestamps[frame_data.channel]
                timestamp0 = math_helper.ts_diff(frame_data.timestamp, frame_data.offset)
                if math_helper.ts_abs_diff_larger_than(timestamp0, prev_timestamp0, MIN_TICKS_BETWEEN_SLOW_BITS):
                    is_full_message = self.ootx_decoder[frame_data.channel].ootx_decoder_process_bit(frame_data.slow_bit)
                self.ootx_timestamps[frame_data.channel] = timestamp0
        # If the OOTX reports to have finished decoding, parse and save the calibration data from it
        if is_full_message:
            self.save_calibration_data()

    def all_calibrations_decoded(self) -> bool:
        """Checks if all calibrations are valid."""
        return all([cal.valid == True for cal in self.base_station_calibrations])

    def save_calibration_data(self) -> None:
        """Parses the cal data from the OOTX decoder and saves it."""
        for bs in range(config.CONFIG_DECK_LIGHTHOUSE_MAX_N_BS):
            if self.ootx_decoder[bs].is_fully_decoded():
                new_cal_data = LighthouseCalibration()
                new_cal_data = self.lighthouse_calibration_init_from_frame(bs)
                if (new_cal_data.uid != self.base_station_calibrations[bs].uid) or (new_cal_data.valid != self.base_station_calibrations[bs].valid):
                    # Received new calibration, update
                    self.base_station_calibrations[bs] = new_cal_data

    def lighthouse_calibration_init_from_frame(self, bs: int) -> LighthouseCalibration:
        """Parses the OOTX data into a calibration frame and returns it."""
        new_cal_data = LighthouseCalibration()
        new_cal_data.sweep[0].phase = self.ootx_decoder[bs].frame.phase0
        new_cal_data.sweep[0].tilt = self.ootx_decoder[bs].frame.tilt0
        new_cal_data.sweep[0].curve = self.ootx_decoder[bs].frame.curve0
        new_cal_data.sweep[0].gibmag = self.ootx_decoder[bs].frame.gibmag0
        new_cal_data.sweep[0].gibphase = self.ootx_decoder[bs].frame.gibphase0
        new_cal_data.sweep[0].ogeemag = self.ootx_decoder[bs].frame.ogeemag0
        new_cal_data.sweep[0].ogeephase = self.ootx_decoder[bs].frame.ogeephase0

        new_cal_data.sweep[1].phase = self.ootx_decoder[bs].frame.phase1
        new_cal_data.sweep[1].tilt = self.ootx_decoder[bs].frame.tilt1
        new_cal_data.sweep[1].curve = self.ootx_decoder[bs].frame.curve1
        new_cal_data.sweep[1].gibmag = self.ootx_decoder[bs].frame.gibmag1
        new_cal_data.sweep[1].gibphase = self.ootx_decoder[bs].frame.gibphase1
        new_cal_data.sweep[1].ogeemag = self.ootx_decoder[bs].frame.ogeemag1
        new_cal_data.sweep[1].ogeephase = self.ootx_decoder[bs].frame.ogeephase1

        new_cal_data.uid = self.ootx_decoder[bs].frame.id
        new_cal_data.valid = True
        return new_cal_data


    # def apply_calibration(self, bs: int):
    #     """Applies the base station calibration to the measured angles."""
    #     do_apply_calibration = self.base_station_calibrations[bs].valid
    #     if do_apply_calibration:
    #         max_delta = 0.0005
    #         for sensor in range(PULSE_PROCESSOR_N_SENSORS):
    #             if do_apply_calibration:
    #                 self.angles.base_station_measurements[bs].sensor_measurements[sensor].corrected_angles = self.angles.base_station_measurements[bs].sensor_measurements[sensor].angles
    #                 # Don't know why 5 times, probably to reach the specified delta
    #                 for i in range(5):
    #                     current_distorted_angles = self.ideal_to_distorted(self.angles.base_station_measurements[bs].sensor_measurements[sensor].corrected_angles, self.base_station_calibrations[bs])
    #                     delta0 = self.angles.base_station_measurements[bs].sensor_measurements[sensor].angles[0] - current_distorted_angles[0]
    #                     delta1 = self.angles.base_station_measurements[bs].sensor_measurements[sensor].angles[1] - current_distorted_angles[1]

    #                     self.angles.base_station_measurements[bs].sensor_measurements[sensor].corrected_angles[0] += delta0
    #                     self.angles.base_station_measurements[bs].sensor_measurements[sensor].corrected_angles[1] += delta1

    #                     if abs(delta0) < max_delta and abs(delta1) < max_delta:
    #                         break
    #                     print(f'Corrected angles: {self.angles.base_station_measurements[bs].sensor_measurements[sensor].corrected_angles}')
    #             else:
    #                 self.angles.base_station_measurements[bs].sensor_measurements[sensor].corrected_angles = self.angles.base_station_measurements[bs].sensor_measurements[sensor].angles
    #         return True
    #     return False

    # def ideal_to_distorted(self, ideal: list, calib: LighthouseCalibrationSweep) -> list:
    #     t30 = math_helper.pi / 6
    #     tan30 = 0.5773502691896258

    #     a1 = ideal[0]
    #     a2 = ideal[1]

    #     x = 1.0
    #     y = math_helper.tan((a2 + a1) / 2.0)
    #     z = math_helper.sin((a2 - a1) / (tan30 * (math_helper.cos(a2) * math_helper.cos(a1))))

    #     return [self.apply_lh2_model(x, y, z, -t30, calib[0]), self.apply_lh2_model(x, y, z, t30, calib[1])]

    # def apply_lh2_model(self, x: float, y: float, z: float, t: float, calib: LighthouseCalibrationSweep) -> float:
    #     ax = math_helper.atan2(y, x)
    #     r = math_helper.sqrt(x * x + y * y)

    #     to_clip = z * math_helper.tan(t - calib.tilt) / r
    #     if to_clip < -1.0:
    #         to_clip = -1.0
    #     if to_clip > 1.0:
    #         to_clip = 1.0

    #     base = ax + math_helper.asin(to_clip)
    #     comp_gib = -calib.gibmag * math_helper.cos(ax + calib.gibphase)
    #     return base - (calib.phase + comp_gib)
