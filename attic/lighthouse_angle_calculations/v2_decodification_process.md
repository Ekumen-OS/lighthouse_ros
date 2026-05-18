# Lighthouse V2 Decodification Process (Corrected)

This document provides the **accurate** and **complete** description of how Lighthouse V2 raw timing measurements are decoded into 3D bearing angles.

## Overview

The Lighthouse V2 decodification process converts raw timing offsets from the hardware into azimuth and elevation angles that represent the 3D direction from a base station to a sensor. This is a **4-step process** that accounts for the unique rotating tilted-plane geometry of V2 base stations.

## Physical Geometry

### Base Station Design

Lighthouse V2 base stations use a fundamentally different design than V1:

- **Rotor**: Contains a single rotating drum with **two light-emitting planes**
- **Physical tilt**: Each light plane is tilted at **±30°** (±π/6 radians) from vertical
- **Rotation**: The rotor spins continuously, producing two sweeps per rotation
- **Sweep sequence**:
  - First sweep: Light plane tilted at **-30°** relative to the rotor's rotation axis
  - Second sweep: Light plane tilted at **+30°** relative to the rotor's rotation axis

**IMPORTANT**: The rotor itself is positioned such that its rotation axis is offset by **120°** from the reference direction. This is why the phase angle corrections are ±60° (±π/3), not ±30°.

### Why Two Different Angles?

This is the most common source of confusion:

1. **Physical tilt angle**: ±30° (±π/6) - The actual mechanical tilt of the light planes
2. **Phase angle corrections**: ±60° (±π/3) - Applied to account for **both** the 30° physical tilt **and** the 120° rotor offset

## The 4-Step Decodification Process

### Step 1: Raw Phase Calculation

Convert timing offsets to raw phase angles representing rotor position.

**Inputs**:
- `offset_0`: Timing of first light pulse (24 MHz clock ticks)
- `offset_1`: Timing of second light pulse (24 MHz clock ticks)
- `period`: Rotor period for the base station's channel (24 MHz clock ticks)

**Formula**:
```
phase_0 = (offset_0 / period) × 2π
phase_1 = (offset_1 / period) × 2π
```

**Output**: Phase angles in radians, range [0, 2π)

**Physical meaning**: These represent the rotor's angular position when each light pulse was emitted.

### Step 2: V2 Angle Calculation

Apply corrections to convert rotor phases into V2 angles that account for the tilted geometry.

**Formula**:
```
v2_angle_1 = phase_0 - π + π/3
v2_angle_2 = phase_1 - π - π/3
```

**Breakdown**:
- `-π`: Centers the angle range from [0, 2π) to [-π, π)
- `+π/3` and `-π/3`: Corrections for the 30° physical tilt combined with the 120° rotor offset

**Output**: V2 angles in radians, range approximately [-π, π)

**Physical meaning**: These represent the orientation of each tilted light plane when it illuminated the sensor, relative to the sensor's position.

**Common misconception**: These ±π/3 (±60°) corrections do NOT mean the light planes are tilted at 60°. They account for both the 30° tilt and a 120° rotational offset in the rotor's coordinate system.

### Step 3: V1 Angle Conversion (Plane Intersection Parameterization)

Convert V2 angles to an intermediate representation using plane intersection geometry.

**Formula**:
```
angleH = (v2_angle_1 + v2_angle_2) / 2

angleV = atan2(sin(v2_angle_2 - v2_angle_1),
               tan(π/6) × (cos(v2_angle_1) + cos(v2_angle_2)))
```

**Note**: `tan(π/6) = tan(30°) ≈ 0.5774` - This uses the **physical tilt angle** of the light planes.

**Output**:
- `angleH`: Horizontal plane angle (NOT standard azimuth yet)
- `angleV`: Vertical plane angle (NOT standard elevation yet)

**Physical meaning**:
- The two tilted light sweeps define two geometric planes in 3D space
- The sensor lies on the intersection line of these two planes
- `angleH` and `angleV` parameterize this intersection line

**Why this step?**: This intermediate representation is used in the Crazyflie firmware for compatibility with their V1 codebase. It's a valid mathematical transformation that preserves the 3D geometry.

### Step 4: Spherical Coordinate Conversion

Convert the plane intersection parameterization to standard spherical coordinates.

**Process**:
1. Construct plane normal vectors from angleH and angleV
2. Compute ray direction as cross product of the plane normals
3. Normalize the ray vector
4. Extract spherical coordinates

**Formulas**:
```
plane_a = [sin(angleH), -cos(angleH), 0]
plane_b = [-sin(angleV), 0, cos(angleV)]

raw_ray = plane_b × plane_a  (cross product)
raw_ray = [cos(angleV)×cos(angleH),
           cos(angleV)×sin(angleH),
           sin(angleV)×cos(angleH)]

ray = raw_ray / |raw_ray|  (normalize)

azimuth = atan2(ray_y, ray_x)
elevation = asin(ray_z)
```

**Output**:
- `azimuth`: Standard horizontal angle [-π, π]
- `elevation`: Standard vertical angle, typically [-π/2, π/2]

**Physical meaning**: These are the standard spherical coordinates of the 3D ray from the base station to the sensor.

## Complete Formula Summary

For quick reference, here's the complete process in one place:

```python
import math

# Step 1: Raw phases
phase_0 = (offset_0 / period) * 2 * math.pi
phase_1 = (offset_1 / period) * 2 * math.pi

# Step 2: V2 angles with corrections
v2_angle_1 = phase_0 - math.pi + math.pi / 3.0  # Note: +π/3 for first sweep
v2_angle_2 = phase_1 - math.pi - math.pi / 3.0  # Note: -π/3 for second sweep

# Step 3: V1 angles (plane intersection)
angleH = (v2_angle_1 + v2_angle_2) / 2.0
tant = math.tan(math.pi / 6.0)  # tan(30°) - physical tilt angle
angleV = math.atan2(
    math.sin(v2_angle_2 - v2_angle_1),
    tant * (math.cos(v2_angle_1) + math.cos(v2_angle_2))
)

# Step 4: Spherical coordinates
sin_h = math.sin(angleH)
cos_h = math.cos(angleH)
sin_v = math.sin(angleV)
cos_v = math.cos(angleV)

plane_a = [sin_h, -cos_h, 0.0]
plane_b = [-sin_v, 0.0, cos_v]

# Cross product: plane_b × plane_a
raw_ray = [
    cos_v * cos_h,
    cos_v * sin_h,
    sin_v * cos_h
]

# Normalize
length = math.sqrt(raw_ray[0]**2 + raw_ray[1]**2 + raw_ray[2]**2)
ray = [raw_ray[0]/length, raw_ray[1]/length, raw_ray[2]/length]

# Extract spherical coordinates
azimuth = math.atan2(ray[1], ray[0])
elevation = math.asin(ray[2])
```

## Simplified Formula (Common Error)

Many implementations try to simplify the elevation calculation. **This produces significant errors!**

### ❌ WRONG Formula (produces ~26° error)

```python
# DO NOT USE THIS!
azimuth = ((phase_0 + phase_1) / 2) - math.pi  # This part is actually correct
beta = (phase_1 - phase_0) - 2 * math.pi / 3
elevation = math.atan(math.sin(beta / 2) / math.tan(math.pi / 6))  # WRONG!
```

**Why is this wrong?**
- It uses a simplified approximation that assumes small angles
- It doesn't properly account for the cross-coupling between the tilted planes
- The error can be as large as 26° at moderate elevation angles

### ✅ CORRECT Formula

Always use the full 4-step process shown above, or at minimum:

```python
# Minimum correct formula (combines some steps)
azimuth = ((phase_0 + phase_1) / 2) - math.pi

v2_angle_1 = phase_0 - math.pi + math.pi / 3
v2_angle_2 = phase_1 - math.pi - math.pi / 3
tant = math.tan(math.pi / 6)

elevation = math.atan2(
    math.sin(v2_angle_2 - v2_angle_1),
    tant * (math.cos(v2_angle_1) + math.cos(v2_angle_2))
)
```

**Note**: The azimuth calculation simplifies because the ±π/3 corrections cancel out when averaged. However, you **must** still compute `v2_angle_1` and `v2_angle_2` for the elevation formula.

## Key Insights

### 1. Two Tilt Angles

- **30° (π/6)**: Physical tilt of light planes - used in tan(π/6) in the elevation formula
- **60° (π/3)**: Phase corrections - accounts for 30° tilt + 120° rotor offset

### 2. Azimuth Simplification

The azimuth formula can be simplified because the tilt corrections cancel:
```
azimuth = (v2_angle_1 + v2_angle_2) / 2
        = ((phase_0 - π + π/3) + (phase_1 - π - π/3)) / 2
        = (phase_0 + phase_1) / 2 - π
```

This is why some implementations skip calculating `v2_angle_1` and `v2_angle_2` for azimuth. However, you **must** calculate them for elevation.

### 3. Elevation Does NOT Simplify

The elevation formula involves cosine terms that do NOT cancel:
```
cos(v2_angle_1) + cos(v2_angle_2) ≠ simplified form
```

You must calculate the full formula. Attempts to simplify it produce large errors.

### 4. Intermediate Coordinate Systems

The process uses three different angle representations:
1. **Phase angles** (raw rotor position)
2. **V2 angles** (tilt-corrected rotor orientations)
3. **V1 angles** (plane intersection parameterization)
4. **Spherical angles** (standard azimuth/elevation)

Each transformation is geometrically meaningful and necessary for correctness.

## Testing and Validation

To verify your implementation:

1. **Use test vectors**: See `test_measurement_processor.cpp` for comprehensive test cases
2. **Compare with Crazyflie**: The Crazyflie firmware is the ground truth
3. **Check edge cases**:
   - Zero elevation (horizontal plane)
   - Pure azimuth variations
   - Combined azimuth and elevation
4. **Validate geometry**: The inverse transformation (angles → phases) should round-trip correctly

## Common Pitfalls

1. **Confusing 30° and 60°**: Remember, 30° is physical, 60° is the phase correction
2. **Simplifying elevation**: Don't try to simplify the plane intersection formula
3. **Wrong sign on corrections**: Note that v2_angle_1 uses +π/3, v2_angle_2 uses -π/3
4. **Skipping intermediate steps**: While some steps can be combined, skipping the v2_angle calculation for elevation produces errors
5. **Using wrong tilt constant**: The elevation formula uses tan(π/6) = tan(30°), not tan(π/3)

## Reference Implementation

The authoritative reference is the **Crazyflie firmware**:
- File: `attic/crazyflie-firmware/src/utils/src/lighthouse/pulse_processor_v2.c`
- Function: `calculateAngles()` and `pulseProcessorV2ConvertToV1Angles()`
- Documentation: `attic/crazyflie-firmware/docs/functional-areas/lighthouse/angle_conversion.md`

**All implementations must produce identical results to the Crazyflie firmware.**

## Coordinate System

The output angles are in the **base station's local coordinate frame**:

- **X-axis**: Points forward from the base station
- **Y-axis**: Points left (from base station's perspective)
- **Z-axis**: Points up
- **Origin**: At the base station's optical center

To use these angles for positioning, you need:
1. Base station calibration data (factory calibration)
2. Base station pose in world frame (from system calibration)

## Revision History

- **2026-05-06**: Complete rewrite with corrected tilt angle explanations
- **Previous**: Documentation had errors conflating physical tilt (30°) with phase corrections (60°)
