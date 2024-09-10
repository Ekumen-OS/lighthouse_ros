from dataclasses import dataclass
from lighthouse_calibration import LighthouseCalibration, LighthouseCalibrationSweep, apply_lh2_model
from pulse_processor import PULSE_PROCESSOR_N_SENSORS, PULSE_PROCESSOR_N_SWEEPS
import numpy as np
import math

@dataclass
class SweepAngleMeasurement:
    timestamp: int
    sensor_pos: np.ndarray
    # Base station position and rotation, we will be hardcoding these
    rotor_pos: np.ndarray
    rotor_rot: np.ndarray
    rotor_rot_inv: np.ndarray
    sensor_id: int
    base_station_id: int
    sweep_id: int
    t: float  # t is the tilt angle of the light plane on the rotor
    measured_sweep_angle: float
    std_dev: float
    calib: LighthouseCalibrationSweep
    calibration_measurement_model: function

    def __init__(self):
      self.timestamp = 0
      self.sensor_pos = []
      self.rotor_pos = []
      self.rotor_rot = []
      self.rotor_rot_inv = []
      self.sensor_id = 0
      self.base_station_id = 0
      self.sweep_id = 0
      self.t = 0.0
      self.measured_sweep_angle = 0.0
      self.std_dev = 0.0
      self.calib = LighthouseCalibrationSweep()
      self.calibration_measurement_model = apply_lh2_model

@dataclass
class SensorDeckPositions:
  sensor_positions_in_deck: list
  def __init__(self):
    sensor_pose_w = 0.015 / 2
    sensor_pose_l = 0.03 / 2
    self.sensor_positions_in_deck = [np.array([-sensor_pose_l, sensor_pose_w, 0.0]),
                                     np.array([-sensor_pose_l, -sensor_pose_w, 0.0]),
                                     np.array([sensor_pose_l, sensor_pose_w, 0.0]),
                                     np.array([sensor_pose_l, -sensor_pose_w, 0.0])]

class PoseEstimator():
  def __init__(self) -> None:
    pass

  def estimate_pose_sweeps(self, bs_calibration: LighthouseCalibration, bs_sensor_measurements: list):
    self.estimate_position_sweeps(bs_calibration, bs_sensor_measurements)
    self.estimate_yaw(bs_calibration, bs_sensor_measurements)

  def estimate_position_sweeps(self, bs_calibration: LighthouseCalibration, bs_sensor_measurements: list):
    sweep_info = SweepAngleMeasurement()
    sweep_info.std_dev = 0.001
    sweep_info.rotor_pos = np.array([0, 0, 0])
    sweep_info.rotor_rot = np.array([1, 0, 0], [0, 1, 0], [0, 0, 1])
    sweep_info.rotor_rot_inv = np.array([1, 0, 0], [0, 1, 0], [0, 0, 1])
    sweep_info.base_station_id = bs_calibration.uid

    for sensor in range(PULSE_PROCESSOR_N_SENSORS):
      sweep_info.sensor_id = sensor
      if bs_sensor_measurements[sensor].valid_count == PULSE_PROCESSOR_N_SWEEPS:
        sweep_info.sensor_pos = SensorDeckPositions.sensor_positions_in_deck[sensor]

        sweep_info.measured_sweep_angle = bs_sensor_measurements[sensor].angles[0]
        if sweep_info.measured_sweep_angle != 0:
          sweep_info.t = -math.pi / 6
          sweep_info.calib = bs_calibration.sweep[0]
          sweep_info.sweep_id = 0
          self.enqueue_sweep_angles(sweep_info)
        sweep_info.measured_sweep_angle = bs_sensor_measurements[sensor].angles[1]
        if sweep_info.measured_sweep_angle != 0:
          sweep_info.t = math.pi / 6
          sweep_info.calib = bs_calibration.sweep[1]
          sweep_info.sweep_id = 1
          self.enqueue_sweep_angles(sweep_info)
