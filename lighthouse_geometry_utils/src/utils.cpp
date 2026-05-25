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

#include "lighthouse_geometry_utils/utils.hpp"

#include <Eigen/Core>

namespace lighthouse_geometry_utils
{

// Lighthouse deck sensor geometry (from Crazyflie firmware)
// Distance between sensors in the width direction (Y-axis): 15mm / 2 = 7.5mm
constexpr double kSensorPosW = 0.015 / 2.0;  // 0.0075 m
// Distance between sensors in the length direction (X-axis): 30mm / 2 = 15mm
constexpr double kSensorPosL = 0.030 / 2.0;  // 0.015 m

const std::vector<Eigen::Vector2d> kLighthouseDeckSensorPoses{
  Eigen::Vector2d(-kSensorPosL, kSensorPosW),    // back-left
  Eigen::Vector2d(-kSensorPosL, -kSensorPosW),   // back-right
  Eigen::Vector2d(kSensorPosL, kSensorPosW),     // front-left
  Eigen::Vector2d(kSensorPosL, -kSensorPosW),    // front-right
};

AutoCovDiagonal extractAutoCovDiagonal(const double * cov_matrix)
{
  // The local parameterization maps 7 parameters to 6 DOF.
  // GetCovarianceBlockInTangentSpace returns a 6x6 matrix.
  // The ProductParameterization is (EigenQuaternionParameterization,
  // IdentityParameterization(3)), so the tangent space order is:
  // [0-2] rotation (body-frame perturbation axes), [3-5] translation (x, y, z).
  AutoCovDiagonal auto_cov;
  auto_cov.roll = cov_matrix[0 * 6 + 0];
  auto_cov.pitch = cov_matrix[1 * 6 + 1];
  auto_cov.yaw = cov_matrix[2 * 6 + 2];
  auto_cov.x = cov_matrix[3 * 6 + 3];
  auto_cov.y = cov_matrix[4 * 6 + 4];
  auto_cov.z = cov_matrix[5 * 6 + 5];
  return auto_cov;
}

}  // namespace lighthouse_geometry_utils
