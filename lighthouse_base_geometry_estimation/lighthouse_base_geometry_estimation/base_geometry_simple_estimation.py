# Copyright 2025 Ekumen, Inc.
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

import numpy as np
import cv2 as cv


class BaseGeometrySimpleEstimator:
    """
    Estimates the geometry of a Lighthouse base station using a simplified approach.

    This estimator treats the problem as a Perspective-n-Point (PnP) problem,
    a common task in computer vision. It uses OpenCV's `solvePnP` function
    to determine the position and orientation of a base station relative to the
    sensor deck. For this, we:
        - Consider the Lighthouse base station as an ideal "virtual camera".
        - Consider the four sensors on the deck as a known 3D object.
        - Consider the deck frame of reference as the world frame of reference
    """

    # Coordinate system transforms
    opencv_to_lighthouse = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ]
    )
    lighthouse_to_opencv = np.array(
        [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ]
    )

    # Configuration values
    sensor_x_distance = 0.0296
    sensor_y_distance = 0.0150

    # Sensors pose in world/deck frame of reference in Lighthouse coordinate system
    sensor_poses_deck_frame_lighthouse_cs = np.float32(
        [
            [-sensor_x_distance / 2.0, sensor_y_distance / 2.0, 0.0],
            [-sensor_x_distance / 2.0, -sensor_y_distance / 2.0, 0.0],
            [sensor_x_distance / 2.0, sensor_y_distance / 2.0, 0.0],
            [sensor_x_distance / 2.0, -sensor_y_distance / 2.0, 0.0],
        ]
    )

    # Sensors pose in world/deck frame of reference in OpenCV coordinate system
    sensor_poses_deck_frame_opencv_cs = np.dot(
        sensor_poses_deck_frame_lighthouse_cs, opencv_to_lighthouse
    )

    def estimate_geometry(self, sensor_angles):
        """
        Estimates the base station's pose using OpenCV's solvePnP.

        Note: internally we work with two different coordinate systems: the standard
        robotic coordinate system (X-forward, Y-left, Z-up) and the coordinate system
        expected by OpenCV (X-right, Y-down, Z-forward). The bulk of this code is for
        transforming to/from these two coordinate systems.

        :param sensor_angles: A numpy array of shape (8,) containing the sensor
                              angles in radians:   [s0.elevation, s0.azimuth,
                                                    s1.elevation, s1.azimuth,
                                                    s2.elevation, s2.azimuth,
                                                    s3.elevation, s3.azimuth].
        :return: tuple[bool, numpy.ndarray, numpy.ndarray]
            - bool: True if the solution was found, False otherwise
            - rotation: A numpy array of shape (3, 3) representing the rotation matrix
            - position: A numpy array of shape (3,) representing the translation vector
        """
        # Projection of the sensors to the image plane
        projected_image = -np.tan(
            [
                [sensor_angles[1], sensor_angles[0]],  # s0.azimuth, s0.elevation
                [sensor_angles[3], sensor_angles[2]],  # s1.azimuth, s1.elevation
                [sensor_angles[5], sensor_angles[4]],  # s2.azimuth, s2.elevation
                [sensor_angles[7], sensor_angles[6]],  # s3.azimuth, s3.elevation
            ]
        )
        # Deck pose in camera/bs frame of reference in OpenCV coordinate system
        (
            solution_found,
            deck_rot_vec_cam_frame_opencv_cs,
            deck_trans_vec_cam_frame_opencv_cs,
        ) = cv.solvePnP(
            BaseGeometrySimpleEstimator.sensor_poses_deck_frame_opencv_cs,
            projected_image,
            np.identity(3),  # Camera matrix, using ideal camera
            None,  # Distortion coefficients, no distortion
            flags=cv.SOLVEPNP_ITERATIVE,
        )

        if solution_found:
            # Camera/bs pose in world/deck frame of reference in OpenCV coordinate system
            deck_rot_mat_cam_frame_opencv_cs, _ = cv.Rodrigues(
                deck_rot_vec_cam_frame_opencv_cs
            )
            bs_rot_mat_cam_opencv_cs = np.transpose(deck_rot_mat_cam_frame_opencv_cs)
            bs_trans_vec_cam_opencv_cs = -np.matmul(
                bs_rot_mat_cam_opencv_cs, deck_trans_vec_cam_frame_opencv_cs
            )

            # Camera/bs pose in world/deck frame of reference in Lighthouse coordinate system
            bs_rot_mat_cam_lighthouse_cs = np.dot(
                BaseGeometrySimpleEstimator.opencv_to_lighthouse,
                np.dot(
                    bs_rot_mat_cam_opencv_cs,
                    BaseGeometrySimpleEstimator.lighthouse_to_opencv,
                ),
            )
            bs_trans_vec_cam_lighthouse_cs = np.dot(
                BaseGeometrySimpleEstimator.opencv_to_lighthouse,
                bs_trans_vec_cam_opencv_cs,
            )

        return (
            solution_found,
            bs_rot_mat_cam_lighthouse_cs,
            bs_trans_vec_cam_lighthouse_cs,
        )
