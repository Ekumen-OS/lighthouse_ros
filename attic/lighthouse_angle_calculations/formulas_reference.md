# Formulas Quick Reference

This is a quick reference for all Lighthouse V2 angle calculation formulas.

**IMPORTANT**: This document shows the FULL 4-step process. Steps 1-3 produce intermediate angle representations (angleH, angleV). Step 4 converts to standard spherical coordinates (azimuth, elevation).

## Input Data

- `offset₀`: First sweep timing offset (24 MHz ticks)
- `offset₁`: Second sweep timing offset (24 MHz ticks)
- `period`: Base station rotor period (24 MHz ticks)

## Step 1: Raw Phase Angles

Convert timing to rotation phase (0 to 2π):

```
phase₀ = (offset₀ / period) × 2π
phase₁ = (offset₁ / period) × 2π
```

## Step 2: V2 Angles with Rotor Tilt

Apply ±60° rotor tilt corrections:

```
v2_angle₁ = phase₀ - π + π/3
v2_angle₂ = phase₁ - π - π/3
```

Where:
- `π/3 = 60°` (rotor tilt angle)
- `-π` centers the range to [-π, π)

## Step 3: Intermediate Angles (Plane Intersection Parameterization)

These are intermediate values, NOT final spherical coordinates!

### angleH (Horizontal Plane Angle)

```
angleH = (v2_angle₁ + v2_angle₂) / 2
```

**Simplified form** (rotor tilts cancel):
```
angleH = (phase₀ + phase₁) / 2 - π
```

### angleV (Vertical Plane Angle)

**CORRECT FORMULA** (plane intersection method):

```
tant = tan(π/6)
angleV = atan2(sin(v2_angle₂ - v2_angle₁),
               tant × (cos(v2_angle₁) + cos(v2_angle₂)))
```

**Alternative form** (equivalent):
```
diff = v2_angle₂ - v2_angle₁
angleV = atan2(sin(diff), tant × (cos(v2_angle₁) + cos(v2_angle₂)))
```

**Note**: angleH and angleV are NOT standard azimuth/elevation! They are intermediate values in a plane intersection parameterization. See Step 4 for conversion to spherical coordinates.

## Step 4: Spherical Coordinates Conversion

Convert the plane intersection angles to standard spherical coordinates:

```python
# Compute plane normal vectors
sin_h = sin(angleH)
cos_h = cos(angleH)
sin_v = sin(angleV)
cos_v = cos(angleV)

plane_a = [sin_h, -cos_h, 0.0]
plane_b = [-sin_v, 0.0, cos_v]

# Ray direction is cross product: plane_b × plane_a
raw_ray = [
    plane_b[1] * plane_a[2] - plane_b[2] * plane_a[1],  # cos_v * cos_h
    plane_b[2] * plane_a[0] - plane_b[0] * plane_a[2],  # cos_v * sin_h
    plane_b[0] * plane_a[1] - plane_b[1] * plane_a[0]   # sin_v * cos_h
]

# Normalize the ray
length = sqrt(raw_ray[0]² + raw_ray[1]² + raw_ray[2]²)
ray = [raw_ray[0]/length, raw_ray[1]/length, raw_ray[2]/length]

# Extract spherical coordinates
azimuth = atan2(ray[1], ray[0])      # True azimuth
elevation = asin(ray[2])             # True elevation
```

**Output**: Standard spherical coordinates (azimuth, elevation) in radians

### ❌ WRONG Formula (Found in Several Buggy Implementations)

**DO NOT USE THIS**:
```
beta = (phase₁ - phase₀) - 2π/3
elevation = atan(sin(beta/2) / tan(π/12))
```

**Problems**:
1. Wrong formula for Step 3 (produces 5-26° errors in angleV)
2. Completely skips Step 4 (treats angleV as final elevation)
3. Double error compounds when using for final positioning

The correct approach is:
- Use the atan2 formula for Step 3 (angleV calculation)
- Then perform Step 4 (spherical conversion)

See [implementation_comparison.md](implementation_comparison.md) for detailed error analysis.

## Complete Implementation (Python)

```python
import math

def calculate_bearing(offset_0, offset_1, period):
    """
    Calculate bearing angles from Lighthouse V2 timing data.

    Performs the full 4-step conversion process to produce standard
    spherical coordinates (azimuth, elevation).

    Args:
        offset_0: First sweep offset (24 MHz ticks)
        offset_1: Second sweep offset (24 MHz ticks)
        period: Rotor period (24 MHz ticks)

    Returns:
        (azimuth, elevation) in radians (standard spherical coordinates)
    """
    # Step 1: Raw phases
    phase_0 = (offset_0 / period) * 2.0 * math.pi
    phase_1 = (offset_1 / period) * 2.0 * math.pi

    # Step 2: V2 angles with phase corrections
    v2_angle_1 = phase_0 - math.pi + math.pi / 3.0   # +60° correction
    v2_angle_2 = phase_1 - math.pi - math.pi / 3.0   # -60° correction

    # Step 3: Intermediate angles (plane intersection)
    tant = math.tan(math.pi / 6.0)  # tan(30°) ≈ 0.5774
    angleH = (v2_angle_1 + v2_angle_2) / 2.0
    angleV = math.atan2(
        math.sin(v2_angle_2 - v2_angle_1),
        tant * (math.cos(v2_angle_1) + math.cos(v2_angle_2))
    )

    # Step 4: Convert to standard spherical coordinates
    sin_h = math.sin(angleH)
    cos_h = math.cos(angleH)
    sin_v = math.sin(angleV)
    cos_v = math.cos(angleV)

    # Plane normal vectors
    plane_a = [sin_h, -cos_h, 0.0]
    plane_b = [-sin_v, 0.0, cos_v]

    # Ray direction (cross product)
    raw_ray = [
        plane_b[1] * plane_a[2] - plane_b[2] * plane_a[1],
        plane_b[2] * plane_a[0] - plane_b[0] * plane_a[2],
        plane_b[0] * plane_a[1] - plane_b[1] * plane_a[0]
    ]

    # Normalize
    length = math.sqrt(raw_ray[0]**2 + raw_ray[1]**2 + raw_ray[2]**2)
    ray = [raw_ray[0]/length, raw_ray[1]/length, raw_ray[2]/length]

    # Extract spherical coordinates
    azimuth = math.atan2(ray[1], ray[0])
    elevation = math.asin(ray[2])

    return (azimuth, elevation)
```

## Complete Implementation (C++)

```cpp
#include <cmath>
#include <utility>
#include <array>

std::pair<double, double> calculateBearing(
    double offset_0,
    double offset_1,
    double period)
{
    // Step 1: Raw phases
    const double phase_0 = (offset_0 / period) * 2.0 * M_PI;
    const double phase_1 = (offset_1 / period) * 2.0 * M_PI;

    // Step 2: V2 angles with phase corrections
    const double v2_angle_1 = phase_0 - M_PI + M_PI / 3.0;  // +60° correction
    const double v2_angle_2 = phase_1 - M_PI - M_PI / 3.0;  // -60° correction

    // Step 3: Intermediate angles (plane intersection)
    const double tant = std::tan(M_PI / 6.0);  // tan(30°)
    const double angleH = (v2_angle_1 + v2_angle_2) / 2.0;
    const double angleV = std::atan2(
        std::sin(v2_angle_2 - v2_angle_1),
        tant * (std::cos(v2_angle_1) + std::cos(v2_angle_2))
    );

    // Step 4: Convert to standard spherical coordinates
    const double sin_h = std::sin(angleH);
    const double cos_h = std::cos(angleH);
    const double sin_v = std::sin(angleV);
    const double cos_v = std::cos(angleV);

    // Plane normal vectors
    std::array<double, 3> plane_a = {sin_h, -cos_h, 0.0};
    std::array<double, 3> plane_b = {-sin_v, 0.0, cos_v};

    // Ray direction (cross product)
    std::array<double, 3> raw_ray = {
        plane_b[1] * plane_a[2] - plane_b[2] * plane_a[1],
        plane_b[2] * plane_a[0] - plane_b[0] * plane_a[2],
        plane_b[0] * plane_a[1] - plane_b[1] * plane_a[0]
    };

    // Normalize
    const double length = std::sqrt(
        raw_ray[0] * raw_ray[0] +
        raw_ray[1] * raw_ray[1] +
        raw_ray[2] * raw_ray[2]
    );
    std::array<double, 3> ray = {
        raw_ray[0] / length,
        raw_ray[1] / length,
        raw_ray[2] / length
    };

    // Extract spherical coordinates
    const double azimuth = std::atan2(ray[1], ray[0]);
    const double elevation = std::asin(ray[2]);

    return {azimuth, elevation};
}
```

## Constants

```python
# Clock frequencies
LIGHTHOUSE_V2_BASE_FREQ = 48_000_000  # Hz
PROCESSING_CLOCK_FREQ = 24_000_000     # Hz

# Rotor geometry
PHYSICAL_TILT_ANGLE = math.pi / 6   # 30 degrees (±π/6)
PHASE_CORRECTION_1 = math.pi / 3    # +60 degrees (+π/3)
PHASE_CORRECTION_2 = -math.pi / 3   # -60 degrees (-π/3)

# Note: Phase corrections are ±60° because they account for:
# - Physical tilt of ±30°
# - Plus 120° rotor coordinate offset between the two planes

# Timestamp
TIMESTAMP_MASK = 0x00FFFFFF  # 24-bit counter

# Timing thresholds (24 MHz ticks)
MAX_TIMESTAMP_DIFF_FOR_SWEEP = 0x10000       # ~2.7 ms
MAX_TIMESTAMP_DIFF_FOR_BLOCK_MATCH = 220_000  # ~9.2 ms
```

## Base Station Periods (24 MHz ticks)

```python
PERIODS = {
    0:  479500,  # 959000 / 2
    1:  478500,  # 957000 / 2
    2:  476500,  # 953000 / 2
    3:  474500,  # 949000 / 2
    4:  473500,  # 947000 / 2
    5:  471500,  # 943000 / 2
    6:  470500,  # 941000 / 2
    7:  469500,  # 939000 / 2
    8:  468500,  # 937000 / 2
    9:  464500,  # 929000 / 2
    10: 459500,  # 919000 / 2
    11: 455500,  # 911000 / 2
    12: 453500,  # 907000 / 2
    13: 450500,  # 901000 / 2
    14: 446500,  # 893000 / 2
    15: 443500   # 887000 / 2
}
```

## Output Ranges

- **azimuth**: [-π, π] radians = [-180°, 180°]
  - 0° = straight ahead (base station X-axis)
  - +90° = left (base station Y-axis)
  - ±180° = behind

- **elevation**: typically [-π/2, π/2] radians = [-90°, 90°]
  - 0° = horizontal (base station XY plane)
  - +90° = straight up
  - -90° = straight down

## Conversion to Degrees

```python
azimuth_deg = math.degrees(azimuth)
elevation_deg = math.degrees(elevation)
```

```cpp
double azimuth_deg = azimuth * 180.0 / M_PI;
double elevation_deg = elevation * 180.0 / M_PI;
```

## Mathematical Constants

```python
import math

math.pi       # π ≈ 3.14159265
math.pi / 6   # π/6 = 30° ≈ 0.52359878  (physical tilt)
math.pi / 3   # π/3 = 60° ≈ 1.04719755  (phase correction)
math.pi / 2   # π/2 = 90° ≈ 1.57079633

math.tan(math.pi / 6)   # tan(30°) ≈ 0.57735027  (used in Step 3)
```

## Reference

All formulas derived from:
- **Source**: Crazyflie firmware `pulse_processor_v2.c` and `lighthouse_geometry.c`
- **Process**:
  - Steps 1-3: `pulse_processor_v2.c` (timing → angleH/angleV)
  - Step 4: `lighthouse_geometry.c::lighthouseGeometryGetRay()` (angleH/angleV → ray)
- **Documentation**: `docs/functional-areas/lighthouse/angle_conversion.md`
- **Authority**: Bitcraze (official Lighthouse implementation)

For a detailed explanation of each step, see [v2_decodification_process.md](v2_decodification_process.md).
