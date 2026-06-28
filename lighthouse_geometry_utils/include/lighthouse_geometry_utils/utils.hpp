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

#ifndef LIGHTHOUSE_GEOMETRY_UTILS__UTILS_HPP_
#define LIGHTHOUSE_GEOMETRY_UTILS__UTILS_HPP_

#include <Eigen/Core>

#include <vector>

#include "lighthouse_geometry_utils/datatypes.hpp"

namespace lighthouse_geometry_utils
{

/// 2D positions (XY) of the four sensors in the Lighthouse Deck frame (in
/// meters).
extern const std::vector<Eigen::Vector2d> kLighthouseDeckSensorPoses;

/**
 * @brief Extracts the diagonal elements from a 6x6 covariance matrix.
 *
 * The covariance matrix is expected to be in tangent space order:
 * [0-2] rotation (roll, pitch, yaw), [3-5] translation (x, y, z).
 *
 * @param cov_matrix Pointer to a 6x6 covariance matrix stored in row-major order.
 * @return AutoCovDiagonal structure containing the diagonal variances.
 */
AutoCovDiagonal extractAutoCovDiagonal(const double * cov_matrix);

}  // namespace lighthouse_geometry_utils

#endif  // LIGHTHOUSE_GEOMETRY_UTILS__UTILS_HPP_
