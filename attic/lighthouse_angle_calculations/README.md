# Lighthouse Angle Calculation Documentation

This folder contains comprehensive documentation about how different implementations convert Lighthouse V2 timing data into bearing angles.

## 🚨 CRITICAL BUG FOUND

**Our ROS implementation has a 26° elevation error!**

- **Impact**: Catastrophic positioning errors (~0.88m error at 2m distance)
- **Affected files**: Both Python and C++ implementations
- **Status**: Bug confirmed, fix documented
- **See**: [BUG_REPORT.md](BUG_REPORT.md) for immediate action items

## 📋 Quick Start

1. **Start here**: [v2_decodification_process.md](v2_decodification_process.md) - **NEW!** Complete, corrected guide to V2 decodification
2. **Critical bug info**: [BUG_REPORT.md](BUG_REPORT.md) - Critical bug details and fix (if not already applied)
3. **Step-by-step math**: [timing_to_bearing_conversion.md](timing_to_bearing_conversion.md) - Step-by-step conversion process
4. **Ground truth**: [crazyflie_firmware_implementation.md](crazyflie_firmware_implementation.md) - Authoritative reference
5. **Verify the fix**: Run `./verify_implementations.py`

## 📚 Documentation

### Core Documentation

- **[v2_decodification_process.md](v2_decodification_process.md)** 📐 **NEW!**
  **Complete and corrected guide** to the V2 decodification process
  - Clarifies the difference between physical tilt (30°) and phase corrections (60°)
  - Explains the full 4-step conversion process
  - Addresses common misconceptions and errors

- **[BUG_REPORT.md](BUG_REPORT.md)** ⚠️
  **START HERE** if you have the old buggy implementation! Critical bug report with fix implementation

- **[timing_to_bearing_conversion.md](timing_to_bearing_conversion.md)** 📐
  Complete step-by-step guide: timing offsets → bearing angles

- **[crazyflie_firmware_implementation.md](crazyflie_firmware_implementation.md)** ✅
  Ground truth reference from Crazyflie firmware

- **[implementation_comparison.md](implementation_comparison.md)** 🔍
  Detailed comparison of all implementations and bugs found

### Quick References

- **[formulas_reference.md](formulas_reference.md)** 📝
  Quick lookup of all formulas in Python and C++

- **[lighthouse_v2_protocol.md](lighthouse_v2_protocol.md)** 📡
  Overview of Lighthouse V2 hardware and protocol

### Verification

- **[verify_implementations.py](verify_implementations.py)** 🧪
  Executable script to verify bug and validate fix

## 🎯 Ground Truth

The **Crazyflie firmware** is the authoritative reference implementation:

- **Source**: `attic/crazyflie-firmware/src/utils/src/lighthouse/pulse_processor_v2.c`
- **Docs**: `attic/crazyflie-firmware/docs/functional-areas/lighthouse/`
- **Authority**: Bitcraze (official Lighthouse implementation)

**All other implementations must match the Crazyflie firmware's behavior.**

## 📊 Summary of Findings

### Implementations Status

| Implementation | Location | Azimuth | Elevation | Status |
|---------------|----------|---------|-----------|--------|
| **Crazyflie firmware** | `attic/crazyflie-firmware/...` | ✅ Correct | ✅ Correct | **Ground Truth** |
| **ROS Python** | `attic/lighthouse_ros/...` | ✅ Correct | ❌ 26° error | **NEEDS FIX** |
| **ROS C++** | `lighthouse_protocol_decoder/...` | ✅ Correct | ❌ 26° error | **NEEDS FIX** |
| **decodeV2.py** | `attic/tools/...` | ✅ Correct | ❌ 5° error | Not critical (test tool) |
| **decodeV2_cf.py** | `attic/tools/...` | ✅ Correct | ❌ 5° error | Not critical (test tool) |

### Key Discoveries

1. **Rotor tilt corrections**: V2 base stations have ±60° tilted light planes
2. **Azimuth is correct**: Rotor tilt cancels out in horizontal angle
3. **Elevation is wrong**: Using simplified formula instead of plane intersection method
4. **Coordinate systems differ**: Crazyflie uses angleH/angleV, we use azimuth/elevation, but both produce identical 3D rays when formulas are correct

## 🔧 How to Fix

See [BUG_REPORT.md](BUG_REPORT.md) for complete fix implementation in both Python and C++.

**TL;DR**: Replace the elevation calculation with:

```python
v2_angle_1 = phase_0 - math.pi + math.pi/3
v2_angle_2 = phase_1 - math.pi - math.pi/3
tant = math.tan(math.pi/6)
elevation = math.atan2(
    math.sin(v2_angle_2 - v2_angle_1),
    tant * (math.cos(v2_angle_1) + math.cos(v2_angle_2))
)
```

## 📝 Documentation Corrections (2026-05-06)

Previous versions of this documentation had errors that confused the physical geometry:

**What was wrong**:
1. **Incorrect notation**: Stated rotor tilt as "±60° (±π/6)" when it should be "±60° (±π/3)"
2. **Conflated two different angles**:
   - Physical light plane tilt: ±30° (±π/6)
   - Phase angle corrections: ±60° (±π/3)
3. **Incomplete explanation**: Didn't explain why corrections are 60° when physical tilt is 30°

**What was corrected**:
1. Created comprehensive new guide: [v2_decodification_process.md](v2_decodification_process.md)
2. Fixed notation in [lighthouse_v2_protocol.md](lighthouse_v2_protocol.md)
3. Clarified explanations in [timing_to_bearing_conversion.md](timing_to_bearing_conversion.md)
4. Added clear distinction between:
   - Physical tilt angle (30°) - used in `tan(π/6)` in formulas
   - Phase corrections (60°) - accounts for tilt + rotor offset

**Key insight**: The ±60° (±π/3) corrections in software account for BOTH the ±30° physical tilt AND a 120° rotational offset in the rotor's coordinate system.

## 📞 Questions?

Refer to the detailed documentation in this folder. Each file is self-contained and cross-referenced.
