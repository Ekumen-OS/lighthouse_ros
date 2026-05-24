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

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>

#include "lighthouse_geometry_utils/deck_pose_optimization.hpp"
#include "lighthouse_geometry_utils/station_angles_utils.hpp"
#include "lighthouse_geometry_utils/station_pose_pnp_solver.hpp"
#include "test_helpers.hpp"
#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils::test
{

// V2 rotor period for channel 1: (959000/2) ticks / 24 MHz = ~19.98ms
constexpr double kV2RotorPeriod = (959000.0 / 2.0) / 24e6;

/**
 * @brief Test fixture for parameterized timestamp-to-bearing conversion tests.
 */
struct DeckPoseParams
{
  Eigen::Vector3d position;
  Sophus::SO3d rotation;
  std::string description;
};

class TimestampsToBearingsConversionTest : public ::testing::TestWithParam<DeckPoseParams>
{
};

/**
 * @brief Convert sensor timestamps to bearing angles using V2 decodification pipeline.
 *
 * @param timestamps Array of 8 timestamps: [0-3] first sweep, [4-7] second sweep
 * @param t0 Reference time (phase origin)
 * @param rotor_period Rotor period in seconds
 * @return Pair of arrays: [elevations, azimuths] for each of 4 sensors
 */
std::pair<std::array<double, 4>, std::array<double, 4>>
decode_timestamps_to_bearings(
  const std::array<double, 8> & timestamps,
  double t0,
  double rotor_period)
{
  // Lambda to normalize angle to [0, 2π) using complex number approach
  auto normalize_phase = [](double phase) {
      double normalized = std::atan2(std::sin(phase), std::cos(phase));
      return normalized >= 0.0 ? normalized : normalized + 2.0 * M_PI;
    };

  std::array<double, 4> elevations;
  std::array<double, 4> azimuths;

  // For each sensor, convert timestamps to bearing angles
  for (std::size_t sensor_idx = 0; sensor_idx < 4; ++sensor_idx) {
    // Get timestamps for this sensor (first and second sweep)
    const double timestamp_sweep_0 = timestamps[sensor_idx];
    const double timestamp_sweep_1 = timestamps[sensor_idx + 4];

    // Step 1: Convert timestamps to rotor phase angles (0 to 2π)
    // This is the raw rotor angle: phase = 2π × (t - t0) / period
    const double phase_0 = normalize_phase(2.0 * M_PI * (timestamp_sweep_0 - t0) / rotor_period);
    const double phase_1 = normalize_phase(2.0 * M_PI * (timestamp_sweep_1 - t0) / rotor_period);

    // Step 2: Convert phase angles to V2 angles (with tilt corrections)
    const auto [v2_angle_1, v2_angle_2] =
      lighthouse_geometry_utils::calculateV2Angles(phase_0, phase_1);

    // Step 3: Convert V2 angles to V1 angles (plane intersection parameterization)
    const auto [angleH, angleV] =
      lighthouse_geometry_utils::calculateV1Angles(v2_angle_1, v2_angle_2);

    // Step 4: Convert V1 angles to spherical coordinates (azimuth, elevation)
    const auto [azimuth, elevation] = convertV1AnglesToSpherical(angleH, angleV);

    elevations[sensor_idx] = elevation;
    azimuths[sensor_idx] = azimuth;
  }

  return {elevations, azimuths};
}

/**
 * @brief Test the complete pipeline from timestamps to bearing angles.
 *
 * This test verifies that timestamps from the physics simulation can be
 * correctly decoded using the standard V2 decodification pipeline:
 * timestamps -> phases -> V2 angles -> V1 angles -> spherical bearings
 *
 * This ensures the sweep plane functor produces data compatible with the
 * hardware protocol decoder used in the measurement processor.
 */
TEST_P(TimestampsToBearingsConversionTest, DecodesTimestampsCorrectly)
{
  const auto params = GetParam();

  // Set up station pose in world frame
  // Station at origin, looking in +X direction (standard orientation)
  const Sophus::SE3d station_pose_in_world = Sophus::SE3d(
    Sophus::SO3d(), Eigen::Vector3d(0.0, 0.0, 0.0));

  // Set up deck pose in world frame using test parameters
  const Sophus::SE3d deck_pose_in_world(params.rotation, params.position);

  // Zero velocity (static test)
  const Eigen::Vector3d deck_velocity(0.0, 0.0, 0.0);

  // Initial guess for timestamps at mid-cycle
  // Both sweeps start at period/2 since phase offsets are handled in functor
  const double mid_cycle_guess = kV2RotorPeriod / 2.0;
  std::array<double, 8> timestamps = {
    mid_cycle_guess, mid_cycle_guess, mid_cycle_guess, mid_cycle_guess,
    mid_cycle_guess, mid_cycle_guess, mid_cycle_guess, mid_cycle_guess
  };

  // Solve for actual intersection timestamps using the sweep plane model
  const double t0 = 0.0;
  const bool success = solve_sweep_plane_timestamps(
    t0,
    kV2RotorPeriod,
    deck_pose_in_world,
    deck_velocity,
    station_pose_in_world,
    timestamps  // output: optimized timestamps
  );

  ASSERT_TRUE(success) << "Timestamp optimization failed to converge";

  // Compute expected measurements using the geometric relationship
  const Sophus::SE3d station_in_deck = deck_pose_in_world.inverse() * station_pose_in_world;
  const auto [expected_elevations, expected_azimuths] =
    compute_expected_measurements(station_in_deck);

  // Decode timestamps to bearing angles using standard V2 pipeline
  const auto [decoded_elevations, decoded_azimuths] =
    decode_timestamps_to_bearings(timestamps, t0, kV2RotorPeriod);

  // Verify decoded bearings match expected values from geometric calculation
  for (std::size_t sensor_idx = 0; sensor_idx < 4; ++sensor_idx) {
    const double azimuth = decoded_azimuths[sensor_idx];
    const double elevation = decoded_elevations[sensor_idx];

    // Verify conversions produce finite values
    EXPECT_TRUE(std::isfinite(azimuth)) << "Sensor " << sensor_idx << " azimuth not finite";
    EXPECT_TRUE(std::isfinite(elevation)) << "Sensor " << sensor_idx << " elevation not finite";

    // Compare against expected values from geometric calculation
    EXPECT_NEAR(azimuth, expected_azimuths[sensor_idx], 1e-3)
      << "Sensor " << sensor_idx << " azimuth mismatch";
    EXPECT_NEAR(elevation, expected_elevations[sensor_idx], 1e-3)
      << "Sensor " << sensor_idx << " elevation mismatch";
  }
}

INSTANTIATE_TEST_SUITE_P(
  VariousDeckPoses,
  TimestampsToBearingsConversionTest,
  ::testing::Values(
    // Test case 1: Deck in front of station, facing it (original test case)
    DeckPoseParams{
    Eigen::Vector3d(2.0, 0.1, 0.2),
    Sophus::SO3d::rotY(M_PI),    // Rotated 180° to face station
    "Front, slightly offset, facing station"
  },
    // Test case 2: Deck directly in front, centered
    DeckPoseParams{
    Eigen::Vector3d(1.5, 0.0, 0.0),
    Sophus::SO3d::rotY(M_PI),
    "Front, centered, facing station"
  },
    // Test case 3: Deck to the side with rotation
    DeckPoseParams{
    Eigen::Vector3d(1.0, 1.0, 0.3),
    Sophus::SO3d::rotZ(M_PI / 4.0) * Sophus::SO3d::rotY(M_PI),
    "Side position, 45° rotation"
  },
    // Test case 4: Deck at an angle above station
    DeckPoseParams{
    Eigen::Vector3d(1.2, 0.0, 0.8),
    Sophus::SO3d::rotX(M_PI / 6.0) * Sophus::SO3d::rotY(M_PI),
    "Elevated, tilted 30° around X"
  },
    // Test case 5: Deck with complex orientation
    DeckPoseParams{
    Eigen::Vector3d(1.5, -0.5, 0.4),
    Sophus::SO3d::rotZ(-M_PI / 3.0) * Sophus::SO3d::rotX(M_PI / 8.0) * Sophus::SO3d::rotY(M_PI),
    "Complex orientation: -60° Z, 22.5° X, 180° Y"
  },
    // Test case 6: Far distance, tilted left
    DeckPoseParams{
    Eigen::Vector3d(3.0, 0.2, 0.1),
    Sophus::SO3d::rotZ(M_PI / 5.0) * Sophus::SO3d::rotY(M_PI),
    "Far distance, 36° Z rotation"
  },
    // Test case 7: Close to station, high elevation
    DeckPoseParams{
    Eigen::Vector3d(0.8, 0.1, 1.2),
    Sophus::SO3d::rotX(-M_PI / 4.0) * Sophus::SO3d::rotY(M_PI),
    "Close, high elevation, -45° X tilt"
  },
    // Test case 8: Negative Y position, rolled
    DeckPoseParams{
    Eigen::Vector3d(1.3, -0.8, 0.5),
    Sophus::SO3d::rotZ(-M_PI / 6.0) * Sophus::SO3d::rotX(M_PI / 9.0) * Sophus::SO3d::rotY(M_PI),
    "Negative Y, -30° Z, 20° X, 180° Y"
  },
    // Test case 9: Upper right diagonal
    DeckPoseParams{
    Eigen::Vector3d(1.4, 1.2, 0.9),
    Sophus::SO3d::rotZ(M_PI / 3.0) * Sophus::SO3d::rotX(-M_PI / 12.0) * Sophus::SO3d::rotY(M_PI),
    "Upper right diagonal, 60° Z, -15° X"
  },
    // Test case 10: Near station floor level
    DeckPoseParams{
    Eigen::Vector3d(2.5, -0.3, -0.2),
    Sophus::SO3d::rotZ(M_PI / 10.0) * Sophus::SO3d::rotY(M_PI),
    "Floor level, 18° Z rotation"
  },
    // Test case 11: Extreme tilt combination
    DeckPoseParams{
    Eigen::Vector3d(1.8, 0.6, 0.7),
    Sophus::SO3d::rotZ(-M_PI / 2.5) * Sophus::SO3d::rotX(M_PI / 7.0) *
    Sophus::SO3d::rotY(M_PI * 0.95),
    "Extreme tilt: -72° Z, 25.7° X, 171° Y"
  },
    // Test case 12: Lower left corner
    DeckPoseParams{
    Eigen::Vector3d(1.1, -1.1, 0.15),
    Sophus::SO3d::rotZ(-M_PI * 0.75) * Sophus::SO3d::rotY(M_PI),
    "Lower left, -135° Z rotation"
  },
    // Test case 13: Asymmetric position and rotation
    DeckPoseParams{
    Eigen::Vector3d(2.2, 0.85, 0.45),
    Sophus::SO3d::rotZ(M_PI / 7.0) * Sophus::SO3d::rotX(-M_PI / 11.0) *
    Sophus::SO3d::rotY(M_PI * 1.05),
    "Asymmetric: 25.7° Z, -16.4° X, 189° Y"
  },
    // Test case 14: Medium distance, steep downward tilt
    DeckPoseParams{
    Eigen::Vector3d(1.6, -0.4, 1.5),
    Sophus::SO3d::rotX(M_PI / 3.5) * Sophus::SO3d::rotZ(-M_PI / 8.0) * Sophus::SO3d::rotY(M_PI),
    "High position, 51.4° X, -22.5° Z"
  },
    // Test case 15: Random compound rotation
    DeckPoseParams{
    Eigen::Vector3d(2.8, 0.95, 0.35),
    Sophus::SO3d::rotZ(M_PI * 0.42) * Sophus::SO3d::rotX(-M_PI * 0.13) *
    Sophus::SO3d::rotY(M_PI * 0.88),
    "Random: 75.6° Z, -23.4° X, 158.4° Y"
  }
  ),
  [](const ::testing::TestParamInfo<DeckPoseParams> & info) {
    // Generate test name from description
    std::string name = info.param.description;
    // Replace spaces and special characters with underscores
    std::replace_if(
      name.begin(), name.end(),
      [](char c) {return !std::isalnum(c);}, '_');
    return name;
  }
);

/**
 * @brief Test parameters for moving deck pose recovery performance assessment.
 */
struct MovingDeckParams
{
  Eigen::Vector3d position;  ///< Initial position of deck relative to station
  double velocity_x;         ///< Velocity component in X direction (m/s)
  double velocity_y;         ///< Velocity component in Y direction (m/s)
  double velocity_z;         ///< Velocity component in Z direction (m/s)
  std::string description;
};

class MovingDeckPoseRecoveryTest : public ::testing::TestWithParam<MovingDeckParams>
{
};

/**
 * @brief Performance assessment test for pose recovery with moving deck.
 *
 * This test assesses the accuracy of PnP and optimization-based pose recovery
 * methods when the deck is moving at various velocities in 3D space.
 * It does not fail, but prints a performance comparison table.
 */
TEST_P(MovingDeckPoseRecoveryTest, AssessesPoseRecoveryWithMotion)
{
  const auto params = GetParam();

  // Station at origin, looking in +X direction
  const Sophus::SE3d station_pose_in_world = Sophus::SE3d(
    Sophus::SO3d(), Eigen::Vector3d(0.0, 0.0, 0.0));

  // Deck initially at specified position, facing station
  const Sophus::SE3d deck_pose_at_t0(
    Sophus::SO3d::rotY(M_PI),
    params.position);

  const double t0 = 0.0;

  // Create solvers
  StationPosePnPSolver pnp_solver;
  DeckPoseOptimization deck_optimizer(
    {station_pose_in_world},
    {0});  // Station ID 0

  // Print table header
  std::cout << "\n=== Moving Deck Pose Recovery Performance ===" << std::endl;
  std::cout << "Position: (" << params.position.x() << ", "
            << params.position.y() << ", " << params.position.z() << ") m, ";
  std::cout << "Velocity: (" << params.velocity_x << ", " << params.velocity_y << ", " <<
    params.velocity_z << ") m/s\n";
  std::cout << std::string(150, '-') << std::endl;
  std::cout << std::setw(8) << "Scale"
            << std::setw(10) << "Vx (m/s)"
            << std::setw(10) << "Vy (m/s)"
            << std::setw(10) << "Vz (m/s)"
            << " | "
            << std::setw(10) << "PnP X"
            << std::setw(10) << "PnP Y"
            << std::setw(10) << "PnP Z"
            << std::setw(10) << "PnP Norm"
            << " | "
            << std::setw(10) << "Opt X"
            << std::setw(10) << "Opt Y"
            << std::setw(10) << "Opt Z"
            << std::setw(10) << "Opt Norm" << std::endl;
  std::cout << std::string(150, '-') << std::endl;

  // Iterate over 20 velocity scaling factors
  for (int i = 0; i <= 20; ++i) {
    const double scale = static_cast<double>(i) / 20.0;
    const Eigen::Vector3d deck_velocity(
      params.velocity_x * scale,
      params.velocity_y * scale,
      params.velocity_z * scale);

    // Initial guess for timestamps
    const double mid_cycle_guess = kV2RotorPeriod / 2.0;
    std::array<double, 8> timestamps = {
      mid_cycle_guess, mid_cycle_guess, mid_cycle_guess, mid_cycle_guess,
      mid_cycle_guess, mid_cycle_guess, mid_cycle_guess, mid_cycle_guess
    };

    // Solve for timestamps with current velocity
    const bool success = solve_sweep_plane_timestamps(
      t0,
      kV2RotorPeriod,
      deck_pose_at_t0,
      deck_velocity,
      station_pose_in_world,
      timestamps);

    ASSERT_TRUE(success) << "Timestamp optimization failed at scale " << scale;

    // Decode timestamps to bearings
    const auto [elevations, azimuths] =
      decode_timestamps_to_bearings(timestamps, t0, kV2RotorPeriod);

    // Method 1: PnP solver (recovers station in deck frame, then invert)
    const Sophus::SE3d station_in_deck_pnp = pnp_solver.solve(elevations, azimuths);
    const Sophus::SE3d deck_pose_pnp = station_in_deck_pnp.inverse() * station_pose_in_world;

    // Method 2: Deck pose optimization
    DeckPoseOptimization::Sample sample;
    sample.elevations = elevations;
    sample.azimuths = azimuths;
    sample.station_id = 0;
    const auto [deck_pose_opt, _] = deck_optimizer.solve({sample});

    // Compute translation errors (in millimeters)
    const Eigen::Vector3d pnp_error_vec =
      1000.0 * (deck_pose_pnp.translation() - deck_pose_at_t0.translation());
    const Eigen::Vector3d opt_error_vec =
      1000.0 * (deck_pose_opt.translation() - deck_pose_at_t0.translation());
    const double pnp_error_norm = pnp_error_vec.norm();
    const double opt_error_norm = opt_error_vec.norm();

    // Print results
    std::cout << std::fixed << std::setprecision(2)
              << std::setw(8) << scale
              << std::setw(10) << deck_velocity.x()
              << std::setw(10) << deck_velocity.y()
              << std::setw(10) << deck_velocity.z()
              << " | "
              << std::setw(10) << pnp_error_vec.x()
              << std::setw(10) << pnp_error_vec.y()
              << std::setw(10) << pnp_error_vec.z()
              << std::setw(10) << pnp_error_norm
              << " | "
              << std::setw(10) << opt_error_vec.x()
              << std::setw(10) << opt_error_vec.y()
              << std::setw(10) << opt_error_vec.z()
              << std::setw(10) << opt_error_norm << std::endl;
  }

  std::cout << std::string(150, '-') << std::endl;
  std::cout << "Test completed (informational only, no assertions)" << std::endl << std::endl;
}

INSTANTIATE_TEST_SUITE_P(
  VariousVelocities,
  MovingDeckPoseRecoveryTest,
  ::testing::Values(
    // Velocity along +Y axis
    MovingDeckParams{Eigen::Vector3d(3.0, 0.0, 0.0), 0.0, 2.0, 0.0,
      "3m distance, plus 1 m/s along Y"},
    // Velocity along -Y axis
    MovingDeckParams{Eigen::Vector3d(3.0, 0.0, 0.0), 0.0, -2.0, 0.0,
      "3m distance, minus 1 m/s along Y"},
    // Velocity along +Z axis
    MovingDeckParams{Eigen::Vector3d(3.0, 0.0, 0.0), 0.0, 0.0, 2.0,
      "3m distance, plus 1 m/s along Z"},
    // Velocity along -Z axis
    MovingDeckParams{Eigen::Vector3d(3.0, 0.0, 0.0), 0.0, 0.0, -2.0,
      "3m distance, minus 1 m/s along Z"},
    // Velocity along both Y and Z axes
    MovingDeckParams{Eigen::Vector3d(3.0, 0.0, 0.0), 0.0, 1.0, 1.0,
      "3m distance, 1 m/s along Y and Z"},
    // Low velocity cases (0.2 m/s)
    MovingDeckParams{Eigen::Vector3d(3.0, 0.0, 0.0), 0.2, 0.0, 0.0,
      "3m distance, plus 0.2 m/s along X"},
    MovingDeckParams{Eigen::Vector3d(3.0, 0.0, 0.0), -0.2, 0.0, 0.0,
      "3m distance, minus 0.2 m/s along X"}),
  [](const ::testing::TestParamInfo<MovingDeckParams> & info) {
    std::string name = info.param.description;
    std::replace_if(
      name.begin(), name.end(),
      [](char c) {return !std::isalnum(c);}, '_');
    return name;
  }
);

/**
 * @brief Test that verifies the 120° rotor separation produces period/3 time offset.
 *
 * For a sensor directly in front of the station (azimuth = 0°), the two sweep
 * planes (mounted 120° apart on the rotor) should produce timestamps separated
 * by approximately period/3 (~6.67ms for a ~20ms period).
 */
TEST(SweepTimeSeparationTest, CenteredSensorHasPeriodThirdOffset)
{
  // Station at origin, looking in +X direction
  const Sophus::SE3d station_pose_in_world = Sophus::SE3d(
    Sophus::SO3d(), Eigen::Vector3d(0.0, 0.0, 0.0));

  // Deck centered directly in front of station at (2.0, 0.0, 0.0)
  // Rotated 180° to face the station (so deck sensors also point toward station)
  const Sophus::SE3d deck_pose_in_world(
    Sophus::SO3d::rotY(M_PI),
    Eigen::Vector3d(2.0, 0.0, 0.0));

  // Static deck (zero velocity)
  const Eigen::Vector3d deck_velocity(0.0, 0.0, 0.0);

  // Initial guess for timestamps
  const double mid_cycle_guess = kV2RotorPeriod / 2.0;
  std::array<double, 8> timestamps = {
    mid_cycle_guess, mid_cycle_guess, mid_cycle_guess, mid_cycle_guess,
    mid_cycle_guess, mid_cycle_guess, mid_cycle_guess, mid_cycle_guess
  };

  // Solve for actual intersection timestamps
  const double t0 = 0.0;
  const bool success = solve_sweep_plane_timestamps(
    t0,
    kV2RotorPeriod,
    deck_pose_in_world,
    deck_velocity,
    station_pose_in_world,
    timestamps);

  ASSERT_TRUE(success) << "Timestamp optimization failed to converge";

  // Expected time separation: 120° of rotation = period/3
  const double expected_time_offset = kV2RotorPeriod / 3.0;

  // For each sensor, verify the time difference between sweeps
  for (std::size_t sensor_idx = 0; sensor_idx < 4; ++sensor_idx) {
    const double timestamp_sweep_0 = timestamps[sensor_idx];
    const double timestamp_sweep_1 = timestamps[sensor_idx + 4];
    const double time_difference = std::abs(timestamp_sweep_1 - timestamp_sweep_0);

    // Allow 1% tolerance for the time offset
    EXPECT_NEAR(time_difference, expected_time_offset, expected_time_offset * 0.01)
      << "Sensor " << sensor_idx
      << " sweep time separation should be ~period/3 (120° rotation)"
      << "\n  Expected: " << expected_time_offset * 1e3 << " ms"
      << "\n  Actual: " << time_difference * 1e3 << " ms"
      << "\n  Difference: " << (time_difference - expected_time_offset) * 1e6 << " μs";
  }

  // Also verify all sensors have approximately the same time offset
  // (they should since deck is centered and facing the station)
  const double first_sensor_offset = std::abs(timestamps[4] - timestamps[0]);
  for (std::size_t sensor_idx = 1; sensor_idx < 4; ++sensor_idx) {
    const double this_sensor_offset = std::abs(timestamps[sensor_idx + 4] - timestamps[sensor_idx]);
    EXPECT_NEAR(this_sensor_offset, first_sensor_offset, 1e-6)
      << "All sensors should have similar time offsets when deck is centered";
  }
}

}  // namespace lighthouse_geometry_utils::test
