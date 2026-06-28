# Message containing the calculated bearing (azimuth and elevation) angles from
# a Bitcraze lighthouse positioning system beacon to each of the sensors on a
# Lighthouse deck.

int8 base_station_id                  # The base station the measurements are coming from
int32 hardware_timestamp      # The timestamp of the measurement
SensorAngles[4] sensor_angles # The angles calculated for each sensor
