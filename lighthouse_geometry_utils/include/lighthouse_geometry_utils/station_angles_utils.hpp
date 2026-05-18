// Copyright 2026 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIGHTHOUSE_GEOMETRY_UTILS__STATION_ANGLES_UTILS_HPP_
#define LIGHTHOUSE_GEOMETRY_UTILS__STATION_ANGLES_UTILS_HPP_

#include <utility>

namespace lighthouse_geometry_utils
{

/**
 * @file station_angles_utils.hpp
 * @brief Lighthouse V2 angle conversion utilities
 *
 * ANGLE CONVENTIONS:
 * ==================
 *
 * 1. Phase Angles (input to calculateV2Angles):
 *    - Range: [0, 2π]
 *    - phase = 0: Rotor pointing backward (-X direction in station frame)
 *    - phase = π: Rotor pointing forward (+X direction in station frame)
 *    - phase = π/2: Rotor pointing left (-Y direction in station frame)
 *    - phase = 3π/2: Rotor pointing right (+Y direction in station frame)
 *
 * 2. V2 Angles (output of calculateV2Angles, input to calculateV1Angles):
 *    - Range: Approximately [-π, π]
 *    - v2_angle = 0: Sweep plane intersecting forward direction
 *    - v2_angle > 0: Sweep plane rotated counterclockwise (when viewed from above)
 *    - v2_angle < 0: Sweep plane rotated clockwise (when viewed from above)
 *    - These angles include tilt corrections for the ±30° physical tilt
 *
 * 3. V1 Angles (output of calculateV1Angles, input to convertV1AnglesToSpherical):
 *    - angleH: Horizontal plane angle (intermediate representation)
 *    - angleV: Vertical plane angle (intermediate representation)
 *    - NOT equivalent to azimuth/elevation - further conversion required
 *
 * 4. Spherical Coordinates (output of convertV1AnglesToSpherical):
 *    - azimuth: Angle in XY plane, from +X axis toward +Y axis
 *    - elevation: Angle above XY plane (positive = above, negative = below)
 *    - Standard spherical coordinate system in station frame
 */

/**
 * @brief Apply V2 rotor tilt corrections to raw phase angles.
 *
 * Transforms raw rotor phase angles to V2 angles that account for the physical
 * tilt of the light planes (±30°) and the 120° separation between sweeps.
 *
 * Phase Convention:
 * - Input phase_0, phase_1: Range [0, 2π], where π = rotor pointing forward
 * - Output v2_angle_1, v2_angle_2: Centered at 0 for forward direction
 *
 * The transformation applies:
 * - v2_angle_1 = phase_0 - π + π/3  (first sweep, -30° tilt)
 * - v2_angle_2 = phase_1 - π - π/3  (second sweep, +30° tilt)
 *
 * The -π term re-centers the coordinate system so that 0 = forward.
 * The ±π/3 terms compensate for both the 30° physical tilt and the
 * 120° angular separation between the two sweeps on the rotor.
 *
 * @param phase_0 First sweep phase angle (radians, 0 to 2π)
 * @param phase_1 Second sweep phase angle (radians, 0 to 2π)
 * @return Pair of corrected V2 angles {v2_angle_1, v2_angle_2} (radians, ≈[-π, π])
 */
std::pair<double, double> calculateV2Angles(double phase_0, double phase_1);

/**
 * @brief Convert V2 angles to V1 plane intersection parameterization.
 *
 * Converts the tilt-corrected V2 angles to the V1 angular representation
 * based on the intersection of two perpendicular planes. This is an intermediate
 * step in the angle decoding pipeline.
 *
 * Implementation follows Crazyflie firmware (pulse_processor_v2.c):
 * - angleH = (v2_angle_1 + v2_angle_2) / 2
 * - angleV computed from plane intersection geometry with 30° tilt angle
 *
 * IMPORTANT: The output angleH and angleV are NOT standard spherical coordinates.
 * They represent a plane-intersection parameterization that must be converted
 * to spherical coordinates using convertV1AnglesToSpherical().
 *
 * @param v2_angle_1 First V2 angle (radians, from calculateV2Angles)
 * @param v2_angle_2 Second V2 angle (radians, from calculateV2Angles)
 * @return Pair of V1 angles {angleH, angleV} (radians, intermediate representation)
 */
std::pair<double, double> calculateV1Angles(double v2_angle_1, double v2_angle_2);

/**
 * @brief Convert V1 plane intersection angles to standard spherical coordinates.
 *
 * Performs the final conversion from V1's plane-intersection representation
 * to standard spherical coordinates (azimuth, elevation) in the station frame.
 *
 * The conversion:
 * 1. Constructs two perpendicular plane normals from angleH and angleV
 * 2. Computes the ray direction as their cross product
 * 3. Converts the ray to spherical coordinates
 *
 * Output Convention (station frame):
 * - azimuth: Angle in XY plane, measured from +X axis toward +Y axis
 *   - azimuth = 0: Ray points in +X direction (forward)
 *   - azimuth = π/2: Ray points in +Y direction (left)
 *   - azimuth = ±π: Ray points in -X direction (backward)
 * - elevation: Angle above/below XY plane
 *   - elevation > 0: Ray points above horizontal plane
 *   - elevation < 0: Ray points below horizontal plane
 *
 * Implementation follows Crazyflie firmware (lighthouse_geometry.c).
 *
 * @param angleH Horizontal V1 angle (radians, from calculateV1Angles)
 * @param angleV Vertical V1 angle (radians, from calculateV1Angles)
 * @return Pair {azimuth, elevation} in standard spherical coordinates (radians)
 */
std::pair<double, double> convertV1AnglesToSpherical(double angleH, double angleV);

}  // namespace lighthouse_geometry_utils

#endif  // LIGHTHOUSE_GEOMETRY_UTILS__STATION_ANGLES_UTILS_HPP_
