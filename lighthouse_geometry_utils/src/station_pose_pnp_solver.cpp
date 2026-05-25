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

  // Evaluate bearing residuals for each IPPE solution
  auto compute_bearing_cost = [&](const Sophus::SE3d & station_in_deck) -> double {
      double total_cost = 0.0;
      for (size_t i = 0; i < 4; ++i) {
        // Sensor position in deck frame (z=0)
        Eigen::Vector3d sensor_in_deck(
          kLighthouseDeckSensorPoses[i].x(),
          kLighthouseDeckSensorPoses[i].y(),
          0.0);

        // Transform to station frame
        const Eigen::Vector3d sensor_in_station =
          station_in_deck.inverse() * sensor_in_deck;

        // Estimated bearing vector
        const Eigen::Vector3d estimated_bearing = sensor_in_station.normalized();

        // Convert observed angles to bearing vector
        const double el = elevations[i];
        const double az = azimuths[i];
        const double cos_el = std::cos(el);
        Eigen::Vector3d observed_bearing(
          cos_el * std::cos(az),
          cos_el * std::sin(az),
          std::sin(el));
        observed_bearing.normalize();

        // Chordal distance residual
        const Eigen::Vector3d residual = estimated_bearing - observed_bearing;
        total_cost += residual.squaredNorm();
      }
      return total_cost;
    };

  // Select solution with lower bearing residual
  size_t best_idx = 0;
  double best_cost = compute_bearing_cost(candidates[0]);
  for (size_t i = 1; i < candidates.size(); ++i) {
    const double cost = compute_bearing_cost(candidates[i]);
    if (cost < best_cost) {
      best_cost = cost;
      best_idx = i;
    }
  }

  // If IPPE gives poor solutions (cost > 1e-6), fall back to plain ITERATIVE
  // This can happen for edge cases like perfectly overhead stations
  if (best_cost > 1e-6) {
    cv::Mat fallback_rvec, fallback_tvec;
    bool fallback_success = cv::solvePnP(
      sensor_poses_3d, image_points_vec, camera_matrix_,
      distortion_coefficients_, fallback_rvec, fallback_tvec,
      false,  // no initial guess
      cv::SOLVEPNP_ITERATIVE);

    if (fallback_success) {
      Sophus::SE3d fallback_candidate = build_station_in_deck(fallback_rvec, fallback_tvec);
      double fallback_cost = compute_bearing_cost(fallback_candidate);
      if (fallback_cost < best_cost) {
        return fallback_candidate;
      }
    }
  }

  // If costs are similar (within 10x), use physical heuristic to disambiguate
  // Step 2b: Disambiguate using physical heuristics if we have 2 solutions
  size_t chosen_idx = best_idx;
  if (candidates.size() > 1) {
    const double cost_ratio = std::max(best_cost, compute_bearing_cost(candidates[1])) /
      std::min(best_cost, compute_bearing_cost(candidates[1]));

    if (cost_ratio < 10.0) {
      // Costs are similar - use physical heuristics to disambiguate
      const Eigen::Vector3d deck_z(0.0, 0.0, 1.0);
      const Eigen::Vector3d station_z0 = candidates[0].rotationMatrix().col(2);
      const Eigen::Vector3d station_z1 = candidates[1].rotationMatrix().col(2);

      const double dot0 = station_z0.dot(deck_z);
      const double dot1 = station_z1.dot(deck_z);

      // Heuristic 1: Prefer solution where Z-axes are aligned (not opposed)
      // This works when station and deck are both roughly upright
      if (dot1 > 0.0 && dot0 <= 0.0) {
        chosen_idx = 1;
      } else if (dot0 > 0.0 && dot1 <= 0.0) {
        chosen_idx = 0;
      } else if (std::abs(dot0) < 0.3 && std::abs(dot1) < 0.3) {
        // Heuristic 2: For ceiling-mounted stations (Z-axes perpendicular),
        // prefer solution where station is "above" the deck
        // Station X-axis points toward target, so it should point down (negative Z)
        // Z-axes are nearly perpendicular - likely ceiling mount
        const Eigen::Vector3d station_x0 = candidates[0].rotationMatrix().col(0);
        const Eigen::Vector3d station_x1 = candidates[1].rotationMatrix().col(0);
        const double x_z0 = station_x0.dot(deck_z);
        const double x_z1 = station_x1.dot(deck_z);

        // Prefer solution where station X-axis points significantly downward
        // This indicates the station is above the deck looking down
        if (x_z1 < x_z0 - 0.1) {
          chosen_idx = 1;
        } else if (x_z0 < x_z1 - 0.1) {
          chosen_idx = 0;
        }
      }
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
    double refined_cost = compute_bearing_cost(refined_candidate);
    double ippe_cost = compute_bearing_cost(candidates[chosen_idx]);

    // Only use refined solution if it's better or very close
    // Allow refinement to be up to 10% worse to account for numerical differences
    if (refined_cost <= ippe_cost * 1.1) {
      return refined_candidate;
    }
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
