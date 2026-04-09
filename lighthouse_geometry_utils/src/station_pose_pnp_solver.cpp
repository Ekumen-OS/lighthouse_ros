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

namespace lighthouse_geometry_utils {

const std::vector<cv::Point3d> StationPosePnPSolver::sensor_poses_in_deck_{
    cv::Point3d(-0.01745, 0.0075, 0.0),  // back-left
    cv::Point3d(-0.01745, -0.0075, 0.0), // back-right
    cv::Point3d(0.01745, 0.0075, 0.0),   // front-left
    cv::Point3d(0.01745, -0.0075, 0.0)   // front-right
};

StationPosePnPSolver::StationPosePnPSolver()
    : camera_matrix_((cv::Mat_<double>(3, 3) << kVirtualPlaneDistanceMeters,
                      0.0, 0.0, 0.0, kVirtualPlaneDistanceMeters, 0.0, 0.0, 0.0,
                      1.0)),
      distortion_coefficients_(cv::Mat::zeros(1, 5, CV_64F)) {}

Sophus::SE3d
StationPosePnPSolver::calculate(const std::array<double, 4> &elevations,
                                const std::array<double, 4> &azimuths) const {
  const auto image_points = projectToVirtualPlane(elevations, azimuths);

  // Convert image points to OpenCV format
  std::vector<cv::Point2d> image_points_vec(image_points.begin(),
                                            image_points.end());

  // Solve PnP to get deck pose in station (camera) frame
  cv::Mat rvec, tvec;
  cv::solvePnP(sensor_poses_in_deck_, image_points_vec, camera_matrix_,
               distortion_coefficients_, rvec, tvec, false,
               cv::SOLVEPNP_ITERATIVE);

  // Convert rotation vector to rotation matrix
  cv::Mat opencv_mat_deck_orientation_in_station_optical;
  cv::Rodrigues(rvec, opencv_mat_deck_orientation_in_station_optical);

  // Extract rotation matrix and translation from OpenCV result
  Eigen::Matrix3d deck_orientation_matrix_in_station_optical;
  deck_orientation_matrix_in_station_optical
      << opencv_mat_deck_orientation_in_station_optical.at<double>(0, 0),
      opencv_mat_deck_orientation_in_station_optical.at<double>(0, 1),
      opencv_mat_deck_orientation_in_station_optical.at<double>(0, 2),
      opencv_mat_deck_orientation_in_station_optical.at<double>(1, 0),
      opencv_mat_deck_orientation_in_station_optical.at<double>(1, 1),
      opencv_mat_deck_orientation_in_station_optical.at<double>(1, 2),
      opencv_mat_deck_orientation_in_station_optical.at<double>(2, 0),
      opencv_mat_deck_orientation_in_station_optical.at<double>(2, 1),
      opencv_mat_deck_orientation_in_station_optical.at<double>(2, 2);

  Eigen::Vector3d deck_translation_in_station_optical(tvec.at<double>(0), //
                                                      tvec.at<double>(1), //
                                                      tvec.at<double>(2));

  // Transform from OpenCV (z forward, y down, x right) optical camera frame to
  // the standard ROS body frame (x forward, y left, z up)
  Eigen::Matrix3d body_orientation_from_optical;
  body_orientation_from_optical << 0, 0, 1, // X_ours from Z_opencv
      -1, 0, 0,                             // Y_ours from -X_opencv
      0, -1, 0;                             // Z_ours from -Y_opencv

  // Apply coordinate transformation to rotation and translation to go from
  // optical frame to body frame
  Eigen::Matrix3d deck_orientation_matrix_in_station_body =
      body_orientation_from_optical *
      deck_orientation_matrix_in_station_optical;
  Eigen::Vector3d deck_translation_in_station_body =
      body_orientation_from_optical * deck_translation_in_station_optical;

  // Build SE3 for deck pose in station body frame
  Sophus::SO3d deck_orientation_in_station_body(
      deck_orientation_matrix_in_station_body);
  Sophus::SE3d deck_pose_in_station_body(deck_orientation_in_station_body,
                                         deck_translation_in_station_body);

  // Invert to get station pose in deck frame
  return deck_pose_in_station_body.inverse();
}

std::array<cv::Point2d, 4> StationPosePnPSolver::projectToVirtualPlane(
    const std::array<double, 4> &elevations,
    const std::array<double, 4> &azimuths) const {
  std::array<cv::Point2d, 4> projected_points;

  for (std::size_t i = 0; i < projected_points.size(); ++i) {
    const double cos_azimuth = std::cos(azimuths[i]);
    const double u = std::tan(azimuths[i]);
    const double v = std::tan(elevations[i]) / cos_azimuth;
    projected_points[i] = cv::Point2d(u, v);
  }

  return projected_points;
}

} // namespace lighthouse_geometry_utils
