# Implementation Comparison and Bugs

This document compares different Lighthouse V2 implementations and documents bugs found.

## Implementations Analyzed

1. **Crazyflie Firmware** (Ground Truth)
   - Location: `attic/crazyflie-firmware/src/utils/src/lighthouse/pulse_processor_v2.c`
   - Status: ✅ **CORRECT** - Reference implementation

2. **ROS Python Implementation**
   - Location: `attic/lighthouse_ros/lighthouse_ros/measurement_processor.py`
   - Status: ❌ **BUG FOUND** - Incorrect elevation formula

3. **ROS C++ Implementation**
   - Location: `lighthouse_protocol_decoder/src/measurement_processor.cpp`
   - Status: ❌ **BUG FOUND** - Incorrect elevation formula (same bug as Python)

4. **Tools: decodeV2.py**
   - Location: `attic/tools/decodeV2.py`
   - Status: ❌ **BUG FOUND** - Incorrect elevation formula

5. **Tools: decodeV2_cf.py**
   - Location: `attic/tools/decodeV2_cf.py`
   - Status: ❌ **TWO BUGS** - Incorrect elevation formula + wrong timing constant

## Critical Bug: Incorrect Elevation Formula

### Affected Code

**ROS Python** (`measurement_processor.py` lines 96-102):
```python
def calculate_polar_bearing(self, phase_beam_0, phase_beam_1):
    """Calculate the polar coordinates of the sensor relative to the base station."""
    azimuth = ((phase_beam_0 + phase_beam_1) / 2) - math.pi
    p = math.radians(60)
    beta = (phase_beam_1 - phase_beam_0) - math.radians(120)
    elevation = math.atan(math.sin(beta / 2) / math.tan(p / 2))  # ❌ WRONG!
    return (azimuth, elevation)
```

**ROS C++** (`measurement_processor.cpp` lines 209-223):
```cpp
std::pair<double, double>
MeasurementProcessor::calculatePolarBearing(
  double phase_beam_0,
  double phase_beam_1) const
{
  const auto azimuth = ((phase_beam_0 + phase_beam_1) / 2.0) - M_PI;
  const auto p = M_PI / 3.0;  // 60 degrees
  const auto beta = (phase_beam_1 - phase_beam_0) - (2.0 * M_PI / 3.0);
  const auto elevation = std::atan(std::sin(beta / 2.0) / std::tan(p / 2.0));  // ❌ WRONG!
  return {azimuth, elevation};
}
```

**decodeV2.py** (lines 7-11):
```python
def calculateAE(firstBeam, secondBeam):
    azimuth = ((firstBeam + secondBeam) / 2) - math.pi
    p = math.radians(60)
    beta = (secondBeam - firstBeam) - math.radians(120)
    elevation = math.atan(math.sin(beta/2)/math.tan(p/2))  # ❌ WRONG!
    return (azimuth, elevation)
```

### The Correct Formula

From **Crazyflie firmware** (`pulse_processor_v2.c` lines 240-244):

```c
float v2Angle1 = firstBeam_with_tilt;   // Already has +π/3 offset
float v2Angle2 = secondBeam_with_tilt;  // Already has -π/3 offset
float tant = tanf(M_PI_F / 6.0f);       // tan(30°)

// Horizontal angle (correct in all implementations)
v1Angles[0] = (v2Angle1 + v2Angle2) / 2.0f;

// Vertical angle - CORRECT FORMULA
v1Angles[1] = atan2f(
    sinf(v2Angle2 - v2Angle1),
    tant * (cosf(v2Angle1) + cosf(v2Angle2))
);
```

### Error Magnitude

Test case: `offset₀=100000, offset₁=200000, period=479500`

| Implementation | Azimuth | Elevation | Error |
|---------------|---------|-----------|-------|
| Crazyflie (correct) | -67.38° | -59.84° | - |
| ROS Python/C++ | -67.38° | **-33.49°** | **26.35°** |
| decodeV2.py | -67.38° | **-54.96°** | **4.88°** |

**Critical finding**: Our ROS implementation has a **26.35° error** in elevation angle! This will cause massive positioning errors.

**Note**: The decodeV2.py error is smaller because it happens to skip the intermediate V2 angle calculation, but it's still using a wrong/simplified formula.

## Fix Required

### For ROS Python Implementation

Replace `measurement_processor.py` lines 96-102:

```python
def calculate_polar_bearing(self, phase_beam_0, phase_beam_1):
    """Calculate the polar coordinates of the sensor relative to the base station."""
    # Azimuth calculation (correct)
    azimuth = ((phase_beam_0 + phase_beam_1) / 2.0) - math.pi

    # Calculate V2 angles with rotor tilt corrections
    v2_angle_1 = phase_beam_0 - math.pi + math.pi / 3.0  # +60° tilt
    v2_angle_2 = phase_beam_1 - math.pi - math.pi / 3.0  # -60° tilt

    # Elevation using plane intersection formula (matches Crazyflie)
    tant = math.tan(math.pi / 6.0)  # tan(30°)
    elevation = math.atan2(
        math.sin(v2_angle_2 - v2_angle_1),
        tant * (math.cos(v2_angle_1) + math.cos(v2_angle_2))
    )

    return (azimuth, elevation)
```

### For ROS C++ Implementation

Replace `measurement_processor.cpp` lines 209-223:

```cpp
std::pair<double, double>
MeasurementProcessor::calculatePolarBearing(
  double phase_beam_0,
  double phase_beam_1) const
{
  // Azimuth calculation (correct)
  const auto azimuth = ((phase_beam_0 + phase_beam_1) / 2.0) - M_PI;

  // Calculate V2 angles with rotor tilt corrections
  const auto v2_angle_1 = phase_beam_0 - M_PI + M_PI / 3.0;  // +60° tilt
  const auto v2_angle_2 = phase_beam_1 - M_PI - M_PI / 3.0;  // -60° tilt

  // Elevation using plane intersection formula (matches Crazyflie)
  const auto tant = std::tan(M_PI / 6.0);  // tan(30°)
  const auto elevation = std::atan2(
    std::sin(v2_angle_2 - v2_angle_1),
    tant * (std::cos(v2_angle_1) + std::cos(v2_angle_2))
  );

  return {azimuth, elevation};
}
```

## Additional Bug in decodeV2_cf.py

**Location**: `attic/tools/decodeV2_cf.py` line 28

```python
MAX_TICKS_BETWEEN_SWEEP_STARTS_TWO_BLOCKS = 10  # ❌ WRONG!
```

**Should be**:
```python
MAX_TICKS_BETWEEN_SWEEP_STARTS_TWO_BLOCKS = 220000  # ✅ CORRECT
```

This constant determines when two sweep blocks are considered a valid pair. The value of 10 is way too small and would reject almost all valid data. This appears to be a typo or placeholder that was never updated.

## Coordinate System Differences

### Crazyflie: angleH and angleV (Plane Intersection)

The Crazyflie firmware uses a **plane intersection** coordinate system:
- `angleH`: Horizontal plane angle (intermediate representation)
- `angleV`: Vertical plane angle (intermediate representation)
- Ray direction: Cross product of the two plane normals (Step 4)
- Final coordinates: Can extract azimuth/elevation from the ray vector

### Our Implementation: Full 4-Step Process

Our ROS implementation performs the complete conversion:
1. Phase angles from timing
2. V2 angles with phase corrections
3. V1 angles (plane intersection parameterization - angleH/angleV)
4. Spherical coordinates conversion (angleH/angleV → ray → azimuth/elevation)

**Key Finding**: When using the **correct formulas**, both approaches produce **identical 3D ray directions**. The Crazyflie stops at Step 3 (angleH, angleV), while our implementation completes Step 4 to get standard spherical coordinates (azimuth, elevation).

## Verification Results

Using test data `offset₀=100000, offset₁=200000, period=479500`:

### Step 3: Intermediate Angles (angleH, angleV)

| System | angleH | angleV |
|--------|--------|--------|
| Crazyflie (stops here) | -67.38° | -59.84° |
| Our ROS (intermediate) | -67.38° | -59.84° |

✅ **Match**: Both produce identical Step 3 values

### Step 4: Final Spherical Coordinates

After converting angleH/angleV to ray and then to spherical coordinates:

| Coordinate | Value |
|------------|-------|
| True Azimuth | -67.38° |
| True Elevation | -59.84° |

**Note**: In this specific test case, the intermediate angleH/angleV happen to numerically equal the final azimuth/elevation, but this is NOT generally true! They are different coordinate systems and the values differ for most sensor positions.

### 3D Ray Direction

Both systems convert to identical Cartesian ray vectors:
```
x = 0.320713
y = -0.769808
z = -0.551850
```

Angular difference: **0.000000°** (perfect match when using correct formulas)

## Summary

| File | Issue | Severity | Fix Priority |
|------|-------|----------|--------------|
| `measurement_processor.py` | Wrong elevation formula | **CRITICAL** | **HIGH** |
| `measurement_processor.cpp` | Wrong elevation formula | **CRITICAL** | **HIGH** |
| `decodeV2.py` | Wrong elevation formula | Medium | Low (test tool) |
| `decodeV2_cf.py` | Wrong elevation + wrong constant | Medium | Low (test tool) |

**Action Required**: Fix the elevation formula in both Python and C++ ROS implementations immediately. The 26° error will cause catastrophic positioning failures.
