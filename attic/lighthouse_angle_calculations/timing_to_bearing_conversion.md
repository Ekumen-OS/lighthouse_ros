# Timing to Bearing Conversion

This document explains how to convert raw timing measurements from Lighthouse V2 sensors into 3D bearing angles.

## Input Data

For each sweep pair, we have:

- `offset₀`: Timing offset of first sweep (6 MHz clock, scaled to 24 MHz)
- `offset₁`: Timing offset of second sweep (6 MHz clock, scaled to 24 MHz)
- `period`: Rotor period for the base station channel (24 MHz ticks)

## Step 1: Calculate Raw Phase Angles

Convert timing offsets to phase angles (0 to 2π representing one full rotation):

```
phase₀ = (offset₀ / period) × 2π
phase₁ = (offset₁ / period) × 2π
```

**Units**: Radians, range [0, 2π)

## Step 2: Apply Phase Angle Corrections (V2 Specific)

The V2 base station has rotating light planes with a physical tilt of ±30°, but the software applies ±60° phase corrections:

```
v2_angle₁ = phase₀ - π + π/3   (first sweep: +60° phase correction)
v2_angle₂ = phase₁ - π - π/3   (second sweep: -60° phase correction)
```

**Physical meaning**: These angles represent the orientation of the tilted light planes when they illuminated the sensor.

**Note**: The `-π` centers the angle range to [-π, π), and the `±π/3` (±60°) accounts for **both** the 30° physical rotor tilt **and** a 120° rotational offset in the rotor's coordinate system.

**Common confusion**: The ±60° (±π/3) corrections do NOT mean the light planes are tilted at 60°. The actual physical tilt is ±30° (±π/6). The 60° value accounts for the combined effect of the physical tilt and rotor geometry.

## Step 3: Convert to Azimuth and Elevation

### Azimuth (Horizontal Angle)

The azimuth is the average of the two V2 angles:

```
azimuth = (v2_angle₁ + v2_angle₂) / 2
```

**Algebraic simplification**:
```
azimuth = ((phase₀ - π + π/3) + (phase₁ - π - π/3)) / 2
        = (phase₀ + phase₁ - 2π) / 2
        = (phase₀ + phase₁) / 2 - π
```

**Important**: The `±π/3` rotor tilt terms **cancel out** in the azimuth calculation! This is why some implementations skip the intermediate V2 angles.

### Elevation (Vertical Angle)

The elevation uses the **plane intersection method** from the Crazyflie firmware:

```
elevation = atan2(sin(v2_angle₂ - v2_angle₁),
                  tan(π/6) × (cos(v2_angle₁) + cos(v2_angle₂)))
```

**Note**: `tan(π/6) = tan(30°) ≈ 0.5774` - This uses the **physical tilt angle** of the light planes (30°), not the phase correction angle (60°).

**Why this formula?**

The two tilted light planes define two geometric planes in 3D space:
- Plane 1 (physical tilt -30°): Normal vector `n₁`
- Plane 2 (physical tilt +30°): Normal vector `n₂`

The sensor lies on the intersection line of these two planes. The elevation angle is derived from the direction of this intersection line.

See `attic/crazyflie-firmware/docs/functional-areas/lighthouse/angle_conversion.md` for the complete geometric derivation.

## Alternative Direct Formula (Equivalent)

Since we can substitute the V2 angle definitions:

```
diff = phase₁ - phase₀ - 2π/3

elevation = atan2(sin(diff),
                  tan(π/6) × (cos(v2_angle₁) + cos(v2_angle₂)))
```

Where:
```
v2_angle₁ = phase₀ - π + π/3
v2_angle₂ = phase₁ - π - π/3
```

**Note**: The cosine terms do NOT simplify, so you must calculate v2_angle₁ and v2_angle₂ for the elevation formula.

## Output

- **azimuth**: Horizontal angle in the base station's XY plane
  - Range: [-π, π)
  - 0° = straight ahead (X-axis)
  - Positive = counterclockwise rotation

- **elevation**: Vertical angle above/below the XY plane
  - Range: typically [-π/2, π/2]
  - 0° = horizontal plane
  - Positive = above plane
  - Negative = below plane

## Coordinate System

The bearing angles are in the **base station's reference frame**:

- **X-axis**: Points forward from base station
- **Y-axis**: Points left (from base station's perspective)
- **Z-axis**: Points up
- **Origin**: At the base station's optical center

To use these bearings for positioning, you need:
1. Base station calibration data (corrects manufacturing imperfections)
2. Base station geometry (position and orientation in world frame)

## Common Pitfalls

### ❌ WRONG: Simplified elevation formula

```python
# This is an APPROXIMATION with ~5° error!
beta = (phase₁ - phase₀) - 2π/3
elevation = atan(sin(beta/2) / tan(π/12))  # WRONG!
```

This formula appears in some reference implementations but produces significant errors.

### ✅ CORRECT: Full geometric formula

```python
v2_angle_1 = phase_0 - math.pi + math.pi/3
v2_angle_2 = phase_1 - math.pi - math.pi/3
tant = math.tan(math.pi/6)

elevation = math.atan2(
    math.sin(v2_angle_2 - v2_angle_1),
    tant * (math.cos(v2_angle_1) + math.cos(v2_angle_2))
)
```

## Example Calculation

Given:
- `offset₀ = 100000` (24 MHz ticks)
- `offset₁ = 200000` (24 MHz ticks)
- `period = 479500` (24 MHz ticks, channel 0)

```python
import math

# Step 1: Raw phases
phase_0 = (100000 / 479500) * 2 * math.pi  # 1.3100 rad = 75.08°
phase_1 = (200000 / 479500) * 2 * math.pi  # 2.6201 rad = 150.16°

# Step 2: V2 angles with tilt corrections
v2_angle_1 = phase_0 - math.pi + math.pi/3  # -0.7837 rad = -44.92°
v2_angle_2 = phase_1 - math.pi - math.pi/3  # -1.5675 rad = -89.84°

# Step 3a: Azimuth (average, tilt cancels)
azimuth = (phase_0 + phase_1)/2 - math.pi  # -1.1756 rad = -67.38°

# Step 3b: Elevation (plane intersection)
tant = math.tan(math.pi/6)  # 0.5774
elevation = math.atan2(
    math.sin(v2_angle_2 - v2_angle_1),  # sin(-44.92°) = -0.7061
    tant * (math.cos(v2_angle_1) + math.cos(v2_angle_2))  # 0.4104
)  # -1.0444 rad = -59.84°
```

**Result**:
- Azimuth: -67.38°
- Elevation: -59.84°
