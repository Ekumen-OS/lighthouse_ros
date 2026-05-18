# Crazyflie Firmware Implementation (Ground Truth)

The Crazyflie firmware is the **authoritative reference implementation** for Lighthouse V2 angle calculations.

## Source Files

Located in `attic/crazyflie-firmware/`:

### Core Implementation

**`src/utils/src/lighthouse/pulse_processor_v2.c`**
- Main V2 pulse processing logic
- Angle calculation functions
- Block matching and validation

**Key functions:**
- `pulseProcessorV2ProcessPulse()` - Process incoming pulses
- `calculateAngles()` - Convert timing to angles (lines 247-260)
- `pulseProcessorV2ConvertToV1Angles()` - Convert V2 to V1 angles (lines 230-244)

### Geometry Calculations

**`src/utils/src/lighthouse/lighthouse_geometry.c`**
- 3D geometric calculations
- Ray vector computation from angles
- Coordinate transformations

**Key function:**
- `lighthouseGeometryGetRay()` - Convert angleH/angleV to 3D ray (lines 128-139)

### Documentation

**`docs/functional-areas/lighthouse/angle_conversion.md`**
- Mathematical derivation of formulas
- LH1 ↔ LH2 angle conversion
- Complete geometric proof

**`docs/functional-areas/lighthouse/system_overview.md`**
- System architecture
- V1 vs V2 differences
- Positioning methods

## Angle Calculation (Ground Truth)

From `pulse_processor_v2.c` lines 247-260:

```c
static void calculateAngles(
    const pulseProcessorV2SweepBlock_t* latestBlock,
    const pulseProcessorV2SweepBlock_t* previousBlock,
    pulseProcessorResult_t* angles)
{
    const uint8_t channel = latestBlock->channel;

    for (int i = 0; i < PULSE_PROCESSOR_N_SENSORS; i++) {
        uint32_t firstOffset = previousBlock->offset[i];
        uint32_t secondOffset = latestBlock->offset[i];
        uint32_t period = CYCLE_PERIODS[channel];

        // Step 1: Calculate V2 angles with rotor tilt corrections
        float firstBeam = (firstOffset * 2 * M_PI_F / period) - M_PI_F + M_PI_F / 3.0f;
        float secondBeam = (secondOffset * 2 * M_PI_F / period) - M_PI_F - M_PI_F / 3.0f;

        // Store intermediate V2 angles
        pulseProcessorSensorMeasurement_t* measurement =
            &angles->baseStationMeasurementsLh2[channel].sensorMeasurements[i];
        measurement->angles[0] = firstBeam;
        measurement->angles[1] = secondBeam;

        // Step 2: Convert to V1-compatible angles (if needed)
        // See pulseProcessorV2ConvertToV1Angles()
    }
}
```

## V2 to V1 Angle Conversion

From `pulse_processor_v2.c` lines 230-244:

```c
void pulseProcessorV2ConvertToV1Angles(
    pulseProcessorResult_t* angles,
    int baseStation,
    int sensor,
    float v1Angles[2])
{
    float v2Angle1 = angles->baseStationMeasurementsLh2[baseStation]
                           .sensorMeasurements[sensor].angles[0];
    float v2Angle2 = angles->baseStationMeasurementsLh2[baseStation]
                           .sensorMeasurements[sensor].angles[1];

    float tant = tanf(M_PI_F / 6.0f);  // tan(30°) = 0.5774

    // Horizontal angle (angleH) - average of V2 angles
    v1Angles[0] = (v2Angle1 + v2Angle2) / 2.0f;

    // Vertical angle (angleV) - plane intersection formula
    v1Angles[1] = atan2f(
        sinf(v2Angle2 - v2Angle1),
        tant * (cosf(v2Angle1) + cosf(v2Angle2))
    );
}
```

## Coordinate System: angleH and angleV

**IMPORTANT**: The Crazyflie uses a **plane intersection parameterization**, NOT standard spherical coordinates!

- **angleH**: Horizontal plane angle (intermediate representation, NOT final azimuth)
- **angleV**: Vertical plane angle (intermediate representation, NOT final elevation)

These intermediate angles must be converted to 3D ray vectors (and then to spherical coordinates if desired) using Step 4.

### Step 4: Converting to 3D Ray Direction

From `lighthouse_geometry.c` lines 128-139:

```c
void lighthouseGeometryGetRay(
    const float angleH,
    const float angleV,
    vec3d ray)
{
    // Normal to X plane (from horizontal angle)
    vec3d a = {sinf(angleH), -cosf(angleH), 0.0f};

    // Normal to Y plane (from vertical angle)
    vec3d b = {-sinf(angleV), 0.0f, cosf(angleV)};

    // Ray is the cross product: b × a
    vec3d raw_ray;
    vec_cross_product(b, a, raw_ray);

    // Normalize
    float len = vec_length(raw_ray);
    vec_scale(raw_ray, 1.0f / len, ray);
}
```

**Geometric interpretation**:
- angleH defines a plane perpendicular to the XY plane
- angleV defines a different plane perpendicular to the XZ plane
- The intersection of these two planes is the ray direction
- This ray can then be converted to standard spherical coordinates (azimuth, elevation)

**Converting ray to spherical coordinates**:
```python
# After getting the ray from lighthouseGeometryGetRay():
azimuth = atan2(ray[1], ray[0])      # Standard spherical azimuth
elevation = asin(ray[2])             # Standard spherical elevation
```

**Key point**: angleH ≠ azimuth and angleV ≠ elevation. They are intermediate values in a different parameterization. Only after converting to the ray vector and extracting spherical coordinates do you get true azimuth/elevation.

## Constants

From `pulse_processor_v2.c`:

```c
// Rotor periods for all 16 channels (in 24 MHz ticks)
static const uint32_t CYCLE_PERIODS[PULSE_PROCESSOR_N_BASE_STATIONS] = {
    959000 / 2,  // Channel 0: 479500
    957000 / 2,  // Channel 1: 478500
    953000 / 2,  // Channel 2: 476500
    949000 / 2,  // Channel 3: 474500
    947000 / 2,  // Channel 4: 473500
    943000 / 2,  // Channel 5: 471500
    941000 / 2,  // Channel 6: 470500
    939000 / 2,  // Channel 7: 469500
    937000 / 2,  // Channel 8: 468500
    929000 / 2,  // Channel 9: 464500
    919000 / 2,  // Channel 10: 459500
    911000 / 2,  // Channel 11: 455500
    907000 / 2,  // Channel 12: 453500
    901000 / 2,  // Channel 13: 450500
    893000 / 2,  // Channel 14: 446500
    887000 / 2   // Channel 15: 443500
};

#define PULSE_PROCESSOR_N_SENSORS 4
#define PULSE_PROCESSOR_N_BASE_STATIONS 16

// Timing thresholds
#define MAX_TICKS_SENSOR_TO_SENSOR 4000
#define MAX_TICKS_BETWEEN_SWEEP_STARTS_TWO_BLOCKS 220000
```

## Key Design Decisions

### 1. Multi-Step Calculation Process

The firmware uses a clear 4-step process:
1. **Phase angles** from timing offsets (`pulseProcessorV2ProcessPulse()`)
2. **V2 angles** with ±π/3 phase corrections (compensate for tilt + rotor offset)
3. **V1 angles** (angleH, angleV) using plane intersection parameterization
4. **Ray vector** via `lighthouseGeometryGetRay()` for 3D positioning

This matches the physical geometry and makes the code easier to understand.

### 2. Plane Intersection Parameterization

Steps 1-3 produce intermediate angles (angleH, angleV) in a plane intersection parameterization. This is NOT standard spherical coordinates! The Crazyflie uses this representation because:
- It directly corresponds to the two tilted rotating planes in the hardware
- The math is simpler for Steps 2-3
- Step 4 converts to 3D ray vectors for actual positioning

### 3. atan2 for Numerical Stability

Uses `atan2()` instead of `atan()` for the angleV calculation to handle all quadrants correctly and avoid division by zero.

## Why This Is Ground Truth

1. **Official implementation**: Developed by Bitcraze (Crazyflie manufacturer) in collaboration with Valve (Lighthouse system creator)
2. **Extensively tested**: Used in thousands of Crazyflie drones with verified positioning accuracy
3. **Well documented**: Complete mathematical derivations in firmware documentation
4. **Matches hardware**: Directly implements the geometric model of the V2 base stations
5. **Production proven**: Years of real-world use in autonomous flight

## Verification

To verify your implementation matches the Crazyflie firmware:

1. Use identical test data (offsets and period)
2. Calculate both angleH and angleV
3. Convert to 3D ray vectors using the plane intersection method
4. Compare ray directions (should be identical to machine precision)

See `implementation_comparison.md` for detailed comparison results.
