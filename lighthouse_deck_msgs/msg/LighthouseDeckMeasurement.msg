# Lighthouse deck measurement message
# Contains angle measurements from lighthouse base stations for multiple sensors

# Header with timestamp of the measurement
std_msgs/Header header

# Station ID for each measurement
# Identifies which base station the measurement came from
int32[] station_id

# Azimuth angles for sensors 0-3 (degrees)
# Each vector corresponds to measurements from one sensor
float64[] azimuth_0
float64[] azimuth_1
float64[] azimuth_2
float64[] azimuth_3

# Elevation angles for sensors 0-3 (degrees)
# Each vector corresponds to measurements from one sensor
float64[] elevation_0
float64[] elevation_1
float64[] elevation_2
float64[] elevation_3

# Note: All vectors must have the same length. The station_id at index i
# corresponds to the angle measurements at index i in all other vectors.
