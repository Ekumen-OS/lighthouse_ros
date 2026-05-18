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

#include "lighthouse_geometry_utils/station_angles_utils.hpp"

#include <array>
#include <cmath>

namespace lighthouse_geometry_utils
{

std::pair<double, double> calculateV2Angles(double phase_0, double phase_1)
{
  // Apply rotor tilt corrections to raw phase angles
  // V2 base stations have light planes tilted ±30° from vertical
  //
  // Phase angle convention (input):
  //   phase = 0: rotor pointing backward (-X)
  //   phase = π: rotor pointing forward (+X)
  //
  // V2 angle convention (output):
  //   v2_angle = 0: sweep plane aligned with forward direction
  //   The -π term re-centers: phase=π → v2_angle≈0
  //   The ±π/3 terms account for 30° tilt and 120° rotor separation
  //
  const auto v2_angle_1 = phase_0 - M_PI + M_PI / 3.0;  // First sweep: -30° tilt
  const auto v2_angle_2 = phase_1 - M_PI - M_PI / 3.0;  // Second sweep: +30° tilt

  return {v2_angle_1, v2_angle_2};
}

std::pair<double, double> calculateV1Angles(double v2_angle_1, double v2_angle_2)
{
  // Convert V2 angles (tilted planes) to V1 angles (plane intersection parameterization)
  // This follows the Crazyflie firmware conversion in pulse_processor_v2.c
  //
  // IMPORTANT: The output angles (angleH, angleV) are NOT spherical coordinates!
  // They represent an intermediate plane-intersection representation that must
  // be further converted using convertV1AnglesToSpherical().
  //
  constexpr double kTiltAngle = M_PI / 6.0;  // 30° - physical tilt of light planes
  const auto tant = std::tan(kTiltAngle);

  // angleH: horizontal plane angle (average of V2 angles)
  const auto angleH = (v2_angle_1 + v2_angle_2) / 2.0;

  // angleV: vertical plane angle (from plane intersection geometry)
  const auto angleV = std::atan2(
    std::sin(v2_angle_2 - v2_angle_1),
    tant * (std::cos(v2_angle_1) + std::cos(v2_angle_2))
  );

  return {angleH, angleV};
}

std::pair<double, double> convertV1AnglesToSpherical(double angleH, double angleV)
{
  // Convert V1 angles (plane intersection) to standard spherical coordinates
  // Following the Crazyflie firmware approach in lighthouse_geometry.c
  //
  // Output spherical coordinates in station frame:
  //   azimuth: angle in XY plane from +X toward +Y (0 = forward)
  //   elevation: angle above XY plane (positive = above, negative = below)
  //

  const auto sin_h = std::sin(angleH);
  const auto cos_h = std::cos(angleH);
  const auto sin_v = std::sin(angleV);
  const auto cos_v = std::cos(angleV);

  // Define normal vectors to two perpendicular planes
  // plane_a: normal to vertical plane rotated by angleH around Z-axis
  // plane_b: normal to horizontal plane rotated by angleV around Y-axis
  const std::array<double, 3> plane_a = {sin_h, -cos_h, 0.0};
  const std::array<double, 3> plane_b = {-sin_v, 0.0, cos_v};

  // Ray direction is the cross product: plane_b × plane_a
  std::array<double, 3> raw_ray = {
    plane_b[1] * plane_a[2] - plane_b[2] * plane_a[1],  // = cos_v * cos_h
    plane_b[2] * plane_a[0] - plane_b[0] * plane_a[2],  // = cos_v * sin_h
    plane_b[0] * plane_a[1] - plane_b[1] * plane_a[0]   // = sin_v * cos_h
  };

  // Normalize the ray vector
  const auto ray_length = std::sqrt(
    raw_ray[0] * raw_ray[0] +
    raw_ray[1] * raw_ray[1] +
    raw_ray[2] * raw_ray[2]
  );

  raw_ray[0] /= ray_length;
  raw_ray[1] /= ray_length;
  raw_ray[2] /= ray_length;

  // Convert normalized ray to true spherical coordinates
  // Standard spherical coordinate conversion: (x, y, z) → (azimuth, elevation)
  const auto azimuth = std::atan2(raw_ray[1], raw_ray[0]);
  const auto elevation = std::asin(raw_ray[2]);

  return {azimuth, elevation};
}

}  // namespace lighthouse_geometry_utils
