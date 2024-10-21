# External
import time
from smbus2 import SMBus

# Internal
from lighthouse_ros.pulse_processor import PulseProcessor
from lighthouse_ros.lighthouse_calibration import LighthouseCalibrator
from lighthouse_ros.serial_handler import SerialHandler, LighthouseUartFrame
from lighthouse_ros_msgs.msg import SensorMeasurements

# Hand testing:
# Create a virtual serial in one terminal: socat -d -d pty,raw,echo=0 pty,raw,echo=0
# Run the core.py script passing the virtual serial port
# Send byte words with: printf '%b' 'bytes' > /dev/pts/...

class LighthouseCore:
    def __init__(self, serial_handler: SerialHandler, publisher, logger):
        # Class variables
        self.pulse_processor = PulseProcessor()
        self.serial_handler = serial_handler
        self.lighthouse_calibrator = LighthouseCalibrator()
        self.publisher = publisher
        self.logger = logger

        try:
            # Write a 0 to get out of the bootloader mode and start receiving data via UART
            i2c_address = 0x2f
            bus = SMBus(1)
            time.sleep(1)
            var = bus.write_byte_data(i2c_address, 0, 0)
        except:
            self.logger.info("Out of bootloader mode")

        # Start the infinite position estimation loop
        self.core_task()

    def core_task(self):
        while(True):
            self.serial_handler.wait_for_sync()
            previous_was_sync_frame = False

            # Start the data aquisition
            is_frame_valid = True
            while is_frame_valid:
                is_frame_valid, lighthouse_uart_frame = self.serial_handler.get_uart_frame_raw()
                # Reset if sync frame
                if lighthouse_uart_frame.is_sync_frame and previous_was_sync_frame:
                    self.pulse_processor.pulse_processor_clear()

                # Check calibrations before processing frames
                if self.lighthouse_calibrator.all_calibrations_decoded():
                    self.process_frames(lighthouse_uart_frame)
                else:
                    # TODO: Calibration disabled until further notice
                    self.logger.warn("Calibration temporarily disabled")
                    # self.lighthouse_calibrator.handle_calibration_data(lighthouse_uart_frame.data)

                previous_was_sync_frame = lighthouse_uart_frame.is_sync_frame

    def process_frames(self, frame: LighthouseUartFrame):
        base_station = int()
        sweep_id = int()

        (result, base_station, sweep_id) = self.pulse_processor.process_pulse(frame.data)
        if result:
            self.use_pulse_result(base_station, sweep_id)
            # DEBUG print
            # self.logger.info(f'Angles: {self.pulse_processor.angles.base_station_measurements[0].sensor_measurements[0].angles}')
            self.pulse_processor.clear_stale_angles()

    def use_pulse_result(self, base_station: int, sweep_id: int):
        if sweep_id == 1:
            calibration_applied, corrected_angles = self.lighthouse_calibrator.apply_calibration(base_station, self.pulse_processor.angles.base_station_measurements[base_station].sensor_measurements)
            if calibration_applied:
                for sensor in range(4):
                    self.pulse_processor.angles.base_station_measurements[base_station].sensor_measurements[sensor].corrected_angles = corrected_angles[sensor]

                # TODO: Firmware here throttles V2 samples, needed?
                # self.throttle()

                self.pulse_processor.clear_outdated(base_station)

                # Send angles to estimator
                sensor_measurement_angle_msg = SensorMeasurements()
                sensor_measurement_angle_msg.base_station_id = base_station
                for sensor in range(4):
                    sensor_measurement_angle_msg.sensor_angles[sensor].angles = self.pulse_processor.angles.base_station_measurements[base_station].sensor_measurements[sensor].corrected_angles
                self.publisher.publish(sensor_measurement_angle_msg)

                # Clear angles after using them to estimate position
                self.pulse_processor.clear_stale_angles()
