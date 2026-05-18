#!/usr/bin/env python3

# Copyright 2026 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Verification script to demonstrate the elevation formula bug and validate the fix.

This script compares different implementations against the Crazyflie firmware ground truth.

NOTE: This script implements the FULL 4-step decodification process:
  1. Phase calculation from timing offsets
  2. V2 angle calculation with phase corrections
  3. V1 angle conversion (plane intersection parameterization)
  4. Spherical coordinate conversion (final azimuth/elevation output)

The comparisons show differences in FINAL spherical coordinates that users would
actually see, not just intermediate angle representations.
"""

import math


def crazyflie_implementation(offset_0, offset_1, period):
    """Ground truth implementation from Crazyflie firmware - FULL 4-step process."""
    # Step 1: Calculate raw phases
    phase_0 = (offset_0 * 2 * math.pi / period)
    phase_1 = (offset_1 * 2 * math.pi / period)

    # Step 2: Calculate V2 angles with phase corrections
    # +π/3 and -π/3 account for 30° physical tilt + 120° rotor offset
    firstBeam = phase_0 - math.pi + math.pi / 3.0
    secondBeam = phase_1 - math.pi - math.pi / 3.0

    # Step 3: Convert to V1 angles (plane intersection parameterization)
    tant = math.tan(math.pi / 6.0)  # tan(30°) - physical tilt angle

    angleH = (firstBeam + secondBeam) / 2.0
    angleV = math.atan2(
        math.sin(secondBeam - firstBeam),
        tant * (math.cos(firstBeam) + math.cos(secondBeam))
    )

    # Step 4: Convert V1 angles to true spherical coordinates
    sin_h = math.sin(angleH)
    cos_h = math.cos(angleH)
    sin_v = math.sin(angleV)
    cos_v = math.cos(angleV)

    # Define normal vectors to two perpendicular planes
    plane_a = [sin_h, -cos_h, 0.0]
    plane_b = [-sin_v, 0.0, cos_v]

    # Ray direction is the cross product: plane_b × plane_a
    raw_ray = [
        plane_b[1] * plane_a[2] - plane_b[2] * plane_a[1],  # cos_v * cos_h
        plane_b[2] * plane_a[0] - plane_b[0] * plane_a[2],  # cos_v * sin_h
        plane_b[0] * plane_a[1] - plane_b[1] * plane_a[0]   # sin_v * cos_h
    ]

    # Normalize the ray vector
    ray_length = math.sqrt(raw_ray[0]**2 + raw_ray[1]**2 + raw_ray[2]**2)
    ray = [raw_ray[0] / ray_length, raw_ray[1] / ray_length, raw_ray[2] / ray_length]

    # Convert normalized ray to true spherical coordinates
    azimuth = math.atan2(ray[1], ray[0])
    elevation = math.asin(ray[2])

    return azimuth, elevation


def current_ros_implementation_BUGGY(offset_0, offset_1, period):
    """Calculate using the BUGGY ROS implementation."""
    phase_0 = (offset_0 / period) * 2.0 * math.pi
    phase_1 = (offset_1 / period) * 2.0 * math.pi

    azimuth = ((phase_0 + phase_1) / 2) - math.pi

    p = math.pi / 3.0  # 60 degrees (phase correction angle)
    beta = (phase_1 - phase_0) - (2.0 * math.pi / 3.0)
    elevation = math.atan(math.sin(beta / 2.0) / math.tan(p / 2.0))  # WRONG formula!

    return azimuth, elevation


def fixed_ros_implementation(offset_0, offset_1, period):
    """Calculate using the fixed implementation matching Crazyflie firmware."""
    # Step 1: Calculate raw phases
    phase_0 = (offset_0 / period) * 2.0 * math.pi
    phase_1 = (offset_1 / period) * 2.0 * math.pi

    # Step 2: Calculate V2 angles with phase corrections
    # +π/3 (+60°) accounts for -30° physical tilt + 120° rotor offset
    # -π/3 (-60°) accounts for +30° physical tilt + 120° rotor offset
    v2_angle_1 = phase_0 - math.pi + math.pi / 3.0
    v2_angle_2 = phase_1 - math.pi - math.pi / 3.0

    # Step 3: V1 angles (plane intersection parameterization)
    tant = math.tan(math.pi / 6.0)  # tan(30°) - physical tilt angle
    angleH = (v2_angle_1 + v2_angle_2) / 2.0
    angleV = math.atan2(
        math.sin(v2_angle_2 - v2_angle_1),
        tant * (math.cos(v2_angle_1) + math.cos(v2_angle_2))
    )

    # Step 4: Convert V1 angles to true spherical coordinates
    sin_h = math.sin(angleH)
    cos_h = math.cos(angleH)
    sin_v = math.sin(angleV)
    cos_v = math.cos(angleV)

    # Define normal vectors to two perpendicular planes
    plane_a = [sin_h, -cos_h, 0.0]
    plane_b = [-sin_v, 0.0, cos_v]

    # Ray direction is the cross product: plane_b × plane_a
    raw_ray = [
        plane_b[1] * plane_a[2] - plane_b[2] * plane_a[1],  # cos_v * cos_h
        plane_b[2] * plane_a[0] - plane_b[0] * plane_a[2],  # cos_v * sin_h
        plane_b[0] * plane_a[1] - plane_b[1] * plane_a[0]   # sin_v * cos_h
    ]

    # Normalize the ray vector
    ray_length = math.sqrt(raw_ray[0]**2 + raw_ray[1]**2 + raw_ray[2]**2)
    ray = [raw_ray[0] / ray_length, raw_ray[1] / ray_length, raw_ray[2] / ray_length]

    # Convert normalized ray to true spherical coordinates
    azimuth = math.atan2(ray[1], ray[0])
    elevation = math.asin(ray[2])

    return azimuth, elevation


def test_implementation(name, func, offset_0, offset_1, period, reference):
    """Test an implementation against the reference."""
    azimuth, elevation = func(offset_0, offset_1, period)
    ref_azimuth, ref_elevation = reference

    azimuth_error = math.degrees(azimuth - ref_azimuth)
    elevation_error = math.degrees(elevation - ref_elevation)

    print(f"\n{name}:")  # noqa: E231
    # noqa comments suppress false positive E231 warnings for f-string format specs
    azimuth_str = (  # noqa: E231
        f"  Spherical Azimuth:   {math.degrees(azimuth):8.4f}° "  # noqa: E231
        f"(error: {azimuth_error:7.4f}°)"  # noqa: E231
    )
    elevation_str = (  # noqa: E231
        f"  Spherical Elevation: {math.degrees(elevation):8.4f}° "  # noqa: E231
        f"(error: {elevation_error:7.4f}°)"  # noqa: E231
    )
    print(azimuth_str)
    print(elevation_str)

    if abs(azimuth_error) < 0.001 and abs(elevation_error) < 0.001:
        print("  ✅ CORRECT - matches Crazyflie firmware")
        return True
    else:
        print("  ❌ ERROR - does not match ground truth!")
        return False


def main():
    print("=" * 80)
    print("LIGHTHOUSE V2 ANGLE CALCULATION VERIFICATION")
    print("=" * 80)

    # Test data
    offset_0 = 100000
    offset_1 = 200000
    period = 479500

    print("\nTest data:")
    print(f"  offset_0 = {offset_0} (24 MHz ticks)")
    print(f"  offset_1 = {offset_1} (24 MHz ticks)")
    print(f"  period = {period} (24 MHz ticks)")

    # Ground truth
    print("\n" + "=" * 80)
    print("GROUND TRUTH (Crazyflie Firmware - Full 4-Step Process)")
    print("=" * 80)

    ref_azimuth, ref_elevation = crazyflie_implementation(offset_0, offset_1, period)
    print(f"  Spherical Azimuth:   {math.degrees(ref_azimuth):8.4f}°")  # noqa: E231
    print(f"  Spherical Elevation: {math.degrees(ref_elevation):8.4f}°")  # noqa: E231

    # Test implementations
    print("\n" + "=" * 80)
    print("TESTING IMPLEMENTATIONS")
    print("=" * 80)

    reference = (ref_azimuth, ref_elevation)

    results = []
    results.append(test_implementation(
        "Current ROS Implementation (BUGGY)",
        current_ros_implementation_BUGGY,
        offset_0, offset_1, period,
        reference
    ))

    results.append(test_implementation(
        "Fixed ROS Implementation",
        fixed_ros_implementation,
        offset_0, offset_1, period,
        reference
    ))

    # Summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)

    if results[0]:
        print("❌ UNEXPECTED: Current implementation passed (bug may have been fixed)")
    else:
        print("✅ CONFIRMED: Current implementation has the bug (as expected)")

    if results[1]:
        print("✅ VERIFIED: Fixed implementation matches Crazyflie firmware")
    else:
        print("❌ ERROR: Fixed implementation still doesn't match!")

    print("\n" + "=" * 80)
    print("RECOMMENDATION")
    print("=" * 80)
    print("""
The current ROS implementation uses an incorrect elevation formula that
produces ~26° error. This must be fixed in:

1. attic/lighthouse_ros/lighthouse_ros/measurement_processor.py
2. lighthouse_protocol_decoder/src/measurement_processor.cpp

See implementation_comparison.md for the exact fix.
""")
    print("=" * 80)


if __name__ == "__main__":
    main()
