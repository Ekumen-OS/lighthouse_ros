from dataclasses import dataclass

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
