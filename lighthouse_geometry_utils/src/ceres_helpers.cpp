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

#include "lighthouse_geometry_utils/ceres_helpers.hpp"

#include <ceres/ceres.h>

namespace lighthouse_geometry_utils
{

ceres::LocalParameterization * createSE3Parameterization()
{
  // Sophus SE3 stores data as [qx, qy, qz, qw, tx, ty, tz] (Eigen quaternion
  // ordering). EigenQuaternionParameterization handles the [x, y, z, w] layout,
  // unlike QuaternionParameterization which expects [w, x, y, z].
  return new ceres::ProductParameterization(
    new ceres::EigenQuaternionParameterization(),
    new ceres::IdentityParameterization(3));
}

}  // namespace lighthouse_geometry_utils
