# Lighthouse V2 Protocol Overview

## Base Station Design

Lighthouse 2.0 base stations use a fundamentally different design than V1:

### Physical Configuration

- **Rotating mechanism**: Contains two tilted light planes
- **Physical light plane tilt**: ±30° (±π/6) from vertical
- **Phase angle corrections** (in software): ±60° (±π/3)
  - First sweep correction: +π/3 (+60°) accounts for -30° physical tilt + 120° rotor offset
  - Second sweep correction: -π/3 (-60°) accounts for +30° physical tilt + 120° rotor offset
- **Rotation axis**: Horizontal (parallel to base station mounting surface)

**Important**: The ±60° corrections applied in software are NOT the same as the ±30° physical tilt of the light planes. The 60° corrections account for both the physical tilt and a 120° rotational offset in the rotor's coordinate system.

### Timing System

- **Main clock**: 48 MHz base frequency
- **Processing clock**: 24 MHz (48 MHz / 2)
- **Timestamp counter**: 24-bit wraparound at 24 MHz
- **Offset measurement**: 6 MHz clock, scaled to 24 MHz by multiplying by 4

### Channels and Periods

V2 base stations use 16 different channels (0-15) with unique rotation periods:

| Channel | Period (ticks @ 24 MHz) | Period (seconds) |
|---------|------------------------|------------------|
| 0       | 959000 / 2 = 479500    | ~19.98 ms        |
| 1       | 957000 / 2 = 478500    | ~19.94 ms        |
| 2       | 953000 / 2 = 476500    | ~19.85 ms        |
| ...     | ...                    | ...              |
| 15      | 887000 / 2 = 443500    | ~18.48 ms        |

**Note**: Periods are divided by 2 because each full rotation produces two sweeps.

### Sweep Sequence

For each full rotation of the rotor:

1. **First sweep** (physical tilt = -30°):
   - Light plane tilted downward at -30° from vertical
   - Sensor detects light pulse
   - Offset₀ measured relative to rotation start
   - Software applies +60° (+π/3) correction to phase

2. **Second sweep** (physical tilt = +30°):
   - Light plane tilted upward at +30° from vertical
   - Sensor detects light pulse
   - Offset₁ measured relative to rotation start
   - Software applies -60° (-π/3) correction to phase

3. The two offsets encode the 3D direction from base station to sensor

## Data Flow

```
Light pulse detected → FPGA measures timing
                    ↓
              24 MHz timestamp (24-bit)
              6 MHz offset (scaled to 24 MHz)
                    ↓
          Decode channel → Get period
                    ↓
      Calculate phase angles (0 to 2π)
                    ↓
      Apply phase corrections (±π/3 = ±60°)
      to account for physical tilt + rotor offset
                    ↓
      Calculate plane intersection geometry
                    ↓
      Convert to bearing angles (azimuth, elevation)
```

## Key Constants

```c
// Clock system
LIGHTHOUSE_V2_BASE_FREQUENCY = 48000000  // Hz
PROCESSING_CLOCK = 24000000               // Hz (48 MHz / 2)

// Timestamp counter
TIMESTAMP_COUNTER_MASK = 0x00FFFFFF       // 24-bit wraparound

// Rotor geometry
PHYSICAL_TILT_ANGLE = π/6                 // 30 degrees - actual light plane tilt
PHASE_CORRECTION = π/3                    // 60 degrees - software correction (tilt + offset)

// Block matching
MAX_TIMESTAMP_DIFF_FOR_SWEEP = 0x10000    // ~2.7 ms @ 24 MHz
MAX_TIMESTAMP_DIFF_FOR_BLOCK_MATCH = 220000  // ~9.2 ms @ 24 MHz
```

**Note on tilt angles**:
- The physical light planes are tilted at ±30° (±π/6)
- The software applies ±60° (±π/3) corrections to account for both the 30° tilt and a 120° rotor coordinate offset
- In formulas: use π/6 (30°) for `tan()` in elevation calculation, but ±π/3 (60°) for phase corrections

## V1 vs V2 Differences

| Feature              | Lighthouse V1          | Lighthouse V2          |
|---------------------|------------------------|------------------------|
| Base stations       | 2 (synchronized)       | Up to 16 (independent) |
| Synchronization     | Required (optical/cable)| None needed           |
| Sweep method        | Linear scan            | Tilted rotating planes |
| Rotor tilt          | 0° (no tilt)           | ±60°                   |
| Channels            | Fixed                  | 16 channels (0-15)     |
| Frame sync          | Required               | Not needed             |

## Protocol Support

- **Mixing V1 and V2**: NOT supported - system must use all V1 or all V2
- **Standard configuration**: 2-3 base stations recommended
- **Extended support**: Up to 16 base stations possible (firmware dependent)
