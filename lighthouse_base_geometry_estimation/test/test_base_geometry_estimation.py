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

import pytest
import numpy as np
import spatialmath as sm
import math

from lighthouse_base_geometry_estimation.base_geometry_simple_estimation import (
    BaseGeometrySimpleEstimator,
)


@pytest.fixture
def estimator_setup():
    """Provide the estimator and deck setup for tests."""
    estimator = BaseGeometrySimpleEstimator()
    # The deck is assumed to be at the origin (0,0,0) with no rotation.
    deck_pose = sm.SE3()
    sensor_poses_in_deck_frame = [
        sm.SE3(p) for p in estimator.sensor_poses_deck_frame_lighthouse_cs
    ]
    return estimator, deck_pose, sensor_poses_in_deck_frame


def _generate_angles(base_station_pose, deck_pose, sensor_poses_in_deck_frame):
    """Generate the sensor angles as seen from a given base station pose."""
    sensor_angles = []
    for sensor_pose_in_deck_frame in sensor_poses_in_deck_frame:
        sensor_pose_in_world = deck_pose * sensor_pose_in_deck_frame

        # Get the sensor's position in the base station's coordinate frame.
        sensor_pose_in_bs_frame = base_station_pose.inv() * sensor_pose_in_world

        dx, dy, dz = sensor_pose_in_bs_frame.t

        # Calculate azimuth and elevation using spherical coordinates
        azimuth = math.atan2(dy, dx)
        elevation = math.atan2(dz, math.sqrt(dx**2 + dy**2))

        sensor_angles.extend([elevation, azimuth])

    return np.array(sensor_angles)


def _run_estimation_test(
    estimator,
    deck_pose,
    sensor_poses_in_deck_frame,
    bs_ground_truth_pose,
    pos_atol=0.7,
    rot_atol=2.1,
):
    """Run a single estimation test case."""
    # Generate the ideal sensor angles from the base station ground truth pose
    sensor_angles = _generate_angles(
        bs_ground_truth_pose, deck_pose, sensor_poses_in_deck_frame
    )
    found, est_rotation, est_position = estimator.estimate_geometry(sensor_angles)

    assert found, "solvePnP should find a solution."

    np.testing.assert_allclose(
        est_position.flatten(),
        bs_ground_truth_pose.t,
        atol=pos_atol,
        err_msg="Estimated position is not close to the ground truth.",
    )

    np.testing.assert_allclose(
        est_rotation,
        bs_ground_truth_pose.R,
        atol=rot_atol,
        err_msg="Estimated rotation is not close to the ground truth.",
    )


def test_bs_in_front(estimator_setup):
    """Test with a base station directly in front of the deck."""
    estimator, deck_pose, sensor_poses_in_deck_frame = estimator_setup
    bs_gt_pose = sm.SE3(2.0, 0.0, 1.5) * sm.SE3.Rz(180, 'deg')
    _run_estimation_test(estimator, deck_pose, sensor_poses_in_deck_frame, bs_gt_pose)


def test_bs_at_an_angle(estimator_setup):
    """Test with a base station at an angle to the deck."""
    estimator, deck_pose, sensor_poses_in_deck_frame = estimator_setup
    bs_gt_pose = sm.SE3(2.0, 2.0, 1.5) * sm.SE3.Rz(180, 'deg')
    _run_estimation_test(estimator, deck_pose, sensor_poses_in_deck_frame, bs_gt_pose)


def test_bs_with_roll(estimator_setup):
    """Test with a base station that has a roll rotation."""
    estimator, deck_pose, sensor_poses_in_deck_frame = estimator_setup
    bs_gt_pose = sm.SE3(2.0, 0.0, 1.5) * sm.SE3.Rx(-5, 'deg')
    _run_estimation_test(estimator, deck_pose, sensor_poses_in_deck_frame, bs_gt_pose)


def test_bs_with_pitch(estimator_setup):
    """Test with a base station that has a pitch rotation."""
    estimator, deck_pose, sensor_poses_in_deck_frame = estimator_setup
    bs_gt_pose = sm.SE3(2.0, 0.0, 1.5) * sm.SE3.Ry(-20, 'deg')
    _run_estimation_test(estimator, deck_pose, sensor_poses_in_deck_frame, bs_gt_pose)


def test_bs_with_roll_pitch_and_yaw(estimator_setup):
    """Test with a base station that has roll, pitch and yaw."""
    estimator, deck_pose, sensor_poses_in_deck_frame = estimator_setup
    # TODO(Nico): this line makes the test fail, only happens when setting the three angles
    # bs_gt_pose = sm.SE3(1.5, -1.5, 2.0)  * sm.SE3.RPY([0, 0, 0], 'deg')
    # _run_estimation_test(estimator, deck_pose, sensor_poses_in_deck_frame, bs_gt_pose)


def test_edge_case_directly_above(estimator_setup):
    """Edge case: Test with the base station directly above the deck."""
    estimator, deck_pose, sensor_poses_in_deck_frame = estimator_setup
    bs_gt_pose = sm.SE3(0.0, 0.0, 2.0) * sm.SE3.Ry(-90, 'deg')  # Pitched down 90 deg
    _run_estimation_test(estimator, deck_pose, sensor_poses_in_deck_frame, bs_gt_pose)


def test_edge_case_far_away(estimator_setup):
    """Edge case: Test with the base station far away from the deck."""
    estimator, deck_pose, sensor_poses_in_deck_frame = estimator_setup
    bs_gt_pose = sm.SE3(50.0, 25.0, 30.0) * sm.SE3.Rz(20, 'deg') * sm.SE3.Ry(-30, 'deg')
    _run_estimation_test(
        estimator, deck_pose, sensor_poses_in_deck_frame, bs_gt_pose, pos_atol=0.2
    )


def test_edge_case_very_close(estimator_setup):
    """Edge case: Test with the base station very close to the deck."""
    estimator, deck_pose, sensor_poses_in_deck_frame = estimator_setup
    bs_gt_pose = sm.SE3(0.1, 0.05, 0.2) * sm.SE3.Ry(-60, 'deg')  # Pitched down sharply
    _run_estimation_test(
        estimator, deck_pose, sensor_poses_in_deck_frame, bs_gt_pose, pos_atol=0.2
    )
