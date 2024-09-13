import math

PULSE_PROCESSOR_TIMESTAMP_BITWIDTH = 24
PULSE_PROCESSOR_TIMESTAMP_MAX = ((1 << PULSE_PROCESSOR_TIMESTAMP_BITWIDTH) - 1)
PULSE_PROCESSOR_TIMESTAMP_BITMASK = PULSE_PROCESSOR_TIMESTAMP_MAX

def calculateAE(firstBeam, secondBeam):
    azimuth = ((firstBeam + secondBeam) / 2) - math.pi
    p = math.radians(60)
    beta = (secondBeam - firstBeam) - math.radians(120)
    elevation = math.atan(math.sin(beta/2)/math.tan(p/2))
    return (azimuth, elevation)

def ts_sub(a, b):
    return (a - b) & 0x00ffffff

def ts_add(a, b):
    return (a + b) & 0x00ffffff

def ts_diff(x, y) -> int:
  return int((x - y)) & PULSE_PROCESSOR_TIMESTAMP_BITMASK

def ts_abs_diff_larger_than(a, b, limit) -> int:
    return ts_diff(a + limit, b) > (limit * 2)
