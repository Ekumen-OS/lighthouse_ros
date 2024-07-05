import math

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
