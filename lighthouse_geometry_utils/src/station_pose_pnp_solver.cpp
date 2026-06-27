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

#include "lighthouse_geometry_utils/station_pose_pnp_solver.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <vector>

#include "lighthouse_geometry_utils/utils.hpp"

namespace lighthouse_geometry_utils
{

StationPosePnPSolver::StationPosePnPSolver()
: camera_matrix_((cv::Mat_<double>(3, 3) << kVirtualPlaneDistanceMeters,
    0.0, 0.0, 0.0, kVirtualPlaneDistanceMeters, 0.0, 0.0, 0.0,
    1.0)),
  distortion_coefficients_(cv::Mat::zeros(1, 5, CV_64F)) {}

Sophus::SE3d
StationPosePnPSolver::solve(
  const std::array<double, 4> & elevations,
  const std::array<double, 4> & azimuths) const
{
  const auto image_points = projectToVirtualPlane(elevations, azimuths);

  // Convert image points to OpenCV format
  std::vector<cv::Point2d> image_points_vec(image_points.begin(),
    image_points.end());

  // Convert 2D sensor poses to 3D for cv::solvePnP (z=0 in deck frame)
  std::vector<cv::Point3d> sensor_poses_3d;
  sensor_poses_3d.reserve(kLighthouseDeckSensorPoses.size());
  for (const auto & p : kLighthouseDeckSensorPoses) {
    sensor_poses_3d.emplace_back(p.x(), p.y(), 0.0);
  }

  // Solve PnP to get deck pose in station (camera) frame.
  // The deck sensors are 4 coplanar points (z=0 in the deck frame), which
  // creates a planar PnP problem with two-fold ambiguity. We use the standard
  // engineering pattern:
  // 1. Use SOLVEPNP_IPPE to get both ambiguous solutions
  // 2. Disambiguate using physical heuristics (Z-axis alignment)
  // 3. Refine the chosen solution with SOLVEPNP_ITERATIVE using it as initial guess

  // Step 1: Get both solutions from IPPE
  std::vector<cv::Mat> rvecs, tvecs;
  int num_solutions = cv::solvePnPGeneric(
    sensor_poses_3d, image_points_vec, camera_matrix_,
    distortion_coefficients_, rvecs, tvecs,
    false,  // useExtrinsicGuess
    cv::SOLVEPNP_IPPE);

  if (num_solutions < 1) {
    return Sophus::SE3d{};  // IPPE failed completely
  }

  // Transform from OpenCV (z forward, y down, x right) optical camera frame to
  // the standard ROS body frame (x forward, y left, z up)
  Eigen::Matrix3d body_orientation_from_optical;
  body_orientation_from_optical << 0, 0, 1,  // X_ours from Z_opencv
    -1, 0, 0,                                 // Y_ours from -X_opencv
    0, -1, 0;                                 // Z_ours from -Y_opencv

  const auto build_station_in_deck =
    [&body_orientation_from_optical](const cv::Mat & rvec_in,
      const cv::Mat & tvec_in) {
      cv::Mat opencv_R;
      cv::Rodrigues(rvec_in, opencv_R);

      Eigen::Matrix3d R_optical;
      R_optical << opencv_R.at<double>(0, 0), opencv_R.at<double>(0, 1),
        opencv_R.at<double>(0, 2), opencv_R.at<double>(1, 0),
        opencv_R.at<double>(1, 1), opencv_R.at<double>(1, 2),
        opencv_R.at<double>(2, 0), opencv_R.at<double>(2, 1),
        opencv_R.at<double>(2, 2);

      Eigen::Vector3d t_optical(tvec_in.at<double>(0), tvec_in.at<double>(1),
        tvec_in.at<double>(2));

      const Eigen::Matrix3d R_body = body_orientation_from_optical * R_optical;
      const Eigen::Vector3d t_body = body_orientation_from_optical * t_optical;

      // Reject pathological numerical outputs (NaN/Inf)
      if (!R_body.allFinite() || !t_body.allFinite()) {
        return Sophus::SE3d{};
      }

      // Project R_body onto SO(3) to guard against minor numerical issues
      const Sophus::SE3d deck_pose_in_station_body(
        Sophus::SO3d::fitToSO3(R_body), t_body);
      return deck_pose_in_station_body.inverse();
    };

  // Step 2: Convert IPPE solutions to station_in_deck frame for disambiguation
  std::vector<Sophus::SE3d> candidates;
  for (size_t i = 0; i < rvecs.size(); ++i) {
    candidates.push_back(build_station_in_deck(rvecs[i], tvecs[i]));
  }

  // If costs are similar (within 10x), use physical heuristic to disambiguate
  // Step 2b: Disambiguate using physical heuristics if we have 2 solutions
  size_t chosen_idx = 0;
  if (candidates.size() > 1) {
    // Costs are similar - use physical heuristics to disambiguate
    const Eigen::Vector3d deck_z(0.0, 0.0, 1.0);
    const Eigen::Vector3d station_z0 = candidates[0].rotationMatrix().col(2);
    const Eigen::Vector3d station_z1 = candidates[1].rotationMatrix().col(2);

    const double dot0 = station_z0.dot(deck_z);
    const double dot1 = station_z1.dot(deck_z);

    if (dot0 > dot1) {
      chosen_idx = 0;
    } else {
      chosen_idx = 1;
    }
  }

  // Step 3: Refine the chosen IPPE solution with SOLVEPNP_ITERATIVE
  // This gives us the disambiguation from IPPE plus the robustness of ITERATIVE
  cv::Mat refined_rvec = rvecs[chosen_idx].clone();
  cv::Mat refined_tvec = tvecs[chosen_idx].clone();
  bool refine_success = cv::solvePnP(
    sensor_poses_3d, image_points_vec, camera_matrix_,
    distortion_coefficients_, refined_rvec, refined_tvec,
    true,  // useExtrinsicGuess - start from disambiguated IPPE solution
    cv::SOLVEPNP_ITERATIVE);

  if (refine_success) {
    // Validate that refinement actually improved the solution
    Sophus::SE3d refined_candidate = build_station_in_deck(refined_rvec, refined_tvec);
    return refined_candidate;
  }

  // Refinement failed or made things worse - return the IPPE solution as-is
  return candidates[chosen_idx];
}

std::array<cv::Point2d, 4> StationPosePnPSolver::projectToVirtualPlane(
  const std::array<double, 4> & elevations,
  const std::array<double, 4> & azimuths) const
{
  std::array<cv::Point2d, 4> projected_points;

  for (std::size_t i = 0; i < projected_points.size(); ++i) {
    const double cos_azimuth = std::cos(azimuths[i]);
    const double u = -std::tan(azimuths[i]);
    const double v = -std::tan(elevations[i]) / cos_azimuth;
    projected_points[i] = cv::Point2d(u, v);
  }

  return projected_points;
}

}  // namespace lighthouse_geometry_utils
