from dataclasses import dataclass
import math

@dataclass
class LighthouseCalibrationSweep:
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
    uid: int
    valid: bool
    sweep: list[LighthouseCalibrationSweep]

    def __init__(self):
        self.uid = 0
        self.valid = False
        self.sweep = [LighthouseCalibrationSweep()] * 2

def apply_lh2_model(x: float, y: float, z: float, t: float, calib: LighthouseCalibrationSweep) -> float:
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
