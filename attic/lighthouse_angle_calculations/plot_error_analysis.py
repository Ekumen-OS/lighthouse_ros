#!/usr/bin/env python3

# Copyright 2026 Ekumen, Inc.
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

"""Plot error analysis for Lighthouse V2 bearing calculations.

Compares the current (buggy) formula to the correct Crazyflie firmware formula.
Generates visualizations showing error magnitude across the full range of
possible bearing angles.

NOTE: This script now implements the FULL 4-step decodification process:
  1. Phase calculation (input to these functions)
  2. V2 angle calculation with phase corrections
  3. V1 angle conversion (plane intersection parameterization)
  4. Spherical coordinate conversion (final azimuth/elevation output)

The error analysis shows the difference in FINAL spherical coordinates that
users would actually see, not just intermediate angle representations.
"""

import math

import matplotlib.pyplot as plt
import numpy as np


def correct_formula(phase_0, phase_1):
    """Correct formula from Crazyflie firmware - FULL 4-step process."""
    # Step 2: V2 angles with phase corrections
    # +π/3 (+60°) accounts for -30° physical tilt + 120° rotor offset
    # -π/3 (-60°) accounts for +30° physical tilt + 120° rotor offset
    v2_angle_1 = phase_0 - math.pi + math.pi / 3.0
    v2_angle_2 = phase_1 - math.pi - math.pi / 3.0

    # Step 3: V1 angles (plane intersection parameterization)
    tant = math.tan(math.pi / 6.0)  # tan(30°) - physical tilt angle
    angleH = (v2_angle_1 + v2_angle_2) / 2.0
    angleV = math.atan2(
        math.sin(v2_angle_2 - v2_angle_1),
        tant * (math.cos(v2_angle_1) + math.cos(v2_angle_2))
    )

    # Step 4: Convert V1 angles to true spherical coordinates
    sin_h = math.sin(angleH)
    cos_h = math.cos(angleH)
    sin_v = math.sin(angleV)
    cos_v = math.cos(angleV)

    # Define normal vectors to two perpendicular planes
    plane_a = [sin_h, -cos_h, 0.0]
    plane_b = [-sin_v, 0.0, cos_v]

    # Ray direction is the cross product: plane_b × plane_a
    raw_ray = [
        plane_b[1] * plane_a[2] - plane_b[2] * plane_a[1],  # cos_v * cos_h
        plane_b[2] * plane_a[0] - plane_b[0] * plane_a[2],  # cos_v * sin_h
        plane_b[0] * plane_a[1] - plane_b[1] * plane_a[0]   # sin_v * cos_h
    ]

    # Normalize the ray vector
    ray_length = math.sqrt(raw_ray[0]**2 + raw_ray[1]**2 + raw_ray[2]**2)
    ray = [raw_ray[0] / ray_length, raw_ray[1] / ray_length, raw_ray[2] / ray_length]

    # Convert normalized ray to true spherical coordinates
    azimuth = math.atan2(ray[1], ray[0])
    elevation = math.asin(ray[2])

    return azimuth, elevation


def buggy_formula(phase_0, phase_1):
    """Calculate using the buggy formula from ROS implementation."""
    azimuth = ((phase_0 + phase_1) / 2.0) - math.pi

    p = math.pi / 3.0  # 60 degrees (phase correction angle)
    beta = (phase_1 - phase_0) - (2.0 * math.pi / 3.0)
    elevation = math.atan(math.sin(beta / 2.0) / math.tan(p / 2.0))  # WRONG formula!

    return azimuth, elevation


def generate_error_data():
    """Generate error data by sampling the phase space."""
    print("Generating error data...")

    # Sample phase angles uniformly
    # Phase ranges from 0 to 2π
    n_samples = 100
    phase_0_range = np.linspace(0, 2 * math.pi, n_samples)
    phase_1_range = np.linspace(0, 2 * math.pi, n_samples)

    # Storage for results
    true_azimuths = []
    true_elevations = []
    azimuth_errors = []
    elevation_errors = []

    for phase_0 in phase_0_range:
        for phase_1 in phase_1_range:
            # Calculate using both formulas
            azimuth_correct, elevation_correct = correct_formula(phase_0, phase_1)
            azimuth_buggy, elevation_buggy = buggy_formula(phase_0, phase_1)

            # Convert to degrees
            azimuth_correct_deg = math.degrees(azimuth_correct)
            elevation_correct_deg = math.degrees(elevation_correct)

            # Only keep points within ±80° range
            if abs(azimuth_correct_deg) <= 80 and abs(elevation_correct_deg) <= 80:
                true_azimuths.append(azimuth_correct_deg)
                true_elevations.append(elevation_correct_deg)

                # Calculate errors
                azimuth_error = math.degrees(azimuth_buggy - azimuth_correct)
                elevation_error = math.degrees(elevation_buggy - elevation_correct)

                azimuth_errors.append(azimuth_error)
                elevation_errors.append(elevation_error)

    print(f"Generated {len(true_azimuths)} data points")
    return (np.array(true_azimuths), np.array(true_elevations),
            np.array(azimuth_errors), np.array(elevation_errors))


def plot_error_analysis(true_azimuths, true_elevations, azimuth_errors, elevation_errors):
    """Create comprehensive error visualization."""
    fig = plt.figure(figsize=(16, 12))

    # ========================================================================
    # 1. Azimuth Error Heatmap
    # ========================================================================
    ax1 = fig.add_subplot(2, 3, 1)
    scatter1 = ax1.scatter(true_azimuths, true_elevations, c=azimuth_errors,
                           cmap='RdYlBu_r', s=2, vmin=-0.5, vmax=0.5)
    ax1.set_xlabel('True Azimuth (deg)', fontsize=10)
    ax1.set_ylabel('True Elevation (deg)', fontsize=10)
    ax1.set_title('Azimuth Error (Spherical Coord)', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linewidth=0.5)
    ax1.axvline(x=0, color='k', linewidth=0.5)
    cbar1 = plt.colorbar(scatter1, ax=ax1)
    cbar1.set_label('Error (deg)', fontsize=9)

    # ========================================================================
    # 2. Elevation Error Heatmap
    # ========================================================================
    ax2 = fig.add_subplot(2, 3, 2)
    scatter2 = ax2.scatter(true_azimuths, true_elevations, c=elevation_errors,
                           cmap='RdYlBu_r', s=2)
    ax2.set_xlabel('True Azimuth (deg)', fontsize=10)
    ax2.set_ylabel('True Elevation (deg)', fontsize=10)
    ax2.set_title('Elevation Error (Spherical Coord)', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linewidth=0.5)
    ax2.axvline(x=0, color='k', linewidth=0.5)
    cbar2 = plt.colorbar(scatter2, ax=ax2)
    cbar2.set_label('Error (deg)', fontsize=9)

    # ========================================================================
    # 3. Total Angular Error
    # ========================================================================
    ax3 = fig.add_subplot(2, 3, 3)
    total_error = np.sqrt(azimuth_errors**2 + elevation_errors**2)
    scatter3 = ax3.scatter(true_azimuths, true_elevations, c=total_error,
                           cmap='YlOrRd', s=2)
    ax3.set_xlabel('True Azimuth (deg)', fontsize=10)
    ax3.set_ylabel('True Elevation (deg)', fontsize=10)
    ax3.set_title('Total Angular Error (deg)', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.axhline(y=0, color='k', linewidth=0.5)
    ax3.axvline(x=0, color='k', linewidth=0.5)
    cbar3 = plt.colorbar(scatter3, ax=ax3)
    cbar3.set_label('Error (deg)', fontsize=9)

    # ========================================================================
    # 4. Azimuth Error vs Elevation
    # ========================================================================
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.scatter(true_elevations, azimuth_errors, c=true_azimuths,
                cmap='viridis', s=1, alpha=0.5)
    ax4.set_xlabel('True Elevation (deg)', fontsize=10)
    ax4.set_ylabel('Azimuth Error (deg)', fontsize=10)
    ax4.set_title('Azimuth Error vs Elevation', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0, color='r', linewidth=1, linestyle='--', alpha=0.7)

    # ========================================================================
    # 5. Elevation Error vs Elevation
    # ========================================================================
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.scatter(true_elevations, elevation_errors, c=true_azimuths,
                cmap='viridis', s=1, alpha=0.5)
    ax5.set_xlabel('True Elevation (deg)', fontsize=10)
    ax5.set_ylabel('Elevation Error (deg)', fontsize=10)
    ax5.set_title('Elevation Error vs Elevation', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    ax5.axhline(y=0, color='r', linewidth=1, linestyle='--', alpha=0.7)

    # ========================================================================
    # 6. Error Statistics
    # ========================================================================
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.axis('off')

    # noqa comments suppress false positive E231 warnings for f-string format specs
    stats_text = f"""
ERROR STATISTICS (±80° range)

Azimuth Error:
  Mean:     {np.mean(azimuth_errors):7.4f}°  # noqa: E231
  Std Dev:  {np.std(azimuth_errors):7.4f}°  # noqa: E231
  Min:      {np.min(azimuth_errors):7.4f}°  # noqa: E231
  Max:      {np.max(azimuth_errors):7.4f}°  # noqa: E231
  RMS:      {np.sqrt(np.mean(azimuth_errors**2)):7.4f}°  # noqa: E231

Elevation Error:
  Mean:     {np.mean(elevation_errors):7.4f}°  # noqa: E231
  Std Dev:  {np.std(elevation_errors):7.4f}°  # noqa: E231
  Min:      {np.min(elevation_errors):7.4f}°  # noqa: E231
  Max:      {np.max(elevation_errors):7.4f}°  # noqa: E231
  RMS:      {np.sqrt(np.mean(elevation_errors**2)):7.4f}°  # noqa: E231

Total Angular Error:
  Mean:     {np.mean(total_error):7.4f}°  # noqa: E231
  Std Dev:  {np.std(total_error):7.4f}°  # noqa: E231
  Min:      {np.min(total_error):7.4f}°  # noqa: E231
  Max:      {np.max(total_error):7.4f}°  # noqa: E231
  RMS:      {np.sqrt(np.mean(total_error**2)):7.4f}°  # noqa: E231

Data Points: {len(true_azimuths)}
"""

    ax6.text(0.1, 0.95, stats_text, transform=ax6.transAxes,
             fontsize=10, verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))

    plt.suptitle(
        'Lighthouse V2 Error Analysis: Buggy vs Correct (Final Spherical Coordinates)',
        fontsize=14, fontweight='bold')
    plt.tight_layout()

    return fig


def plot_3d_error_surface(true_azimuths, true_elevations, elevation_errors):
    """Create 3D surface plot of elevation error."""
    fig = plt.figure(figsize=(14, 6))

    # Create grid for surface plot
    azimuth_bins = np.linspace(-80, 80, 50)
    elevation_bins = np.linspace(-80, 80, 50)

    # Bin the data
    error_grid = np.full((len(elevation_bins)-1, len(azimuth_bins)-1), np.nan)

    for i in range(len(azimuth_bins)-1):
        for j in range(len(elevation_bins)-1):
            mask = ((true_azimuths >= azimuth_bins[i]) &
                    (true_azimuths < azimuth_bins[i+1]) &
                    (true_elevations >= elevation_bins[j]) &
                    (true_elevations < elevation_bins[j+1]))
            if np.any(mask):
                error_grid[j, i] = np.mean(elevation_errors[mask])

    # Create meshgrid for plotting
    X, Y = np.meshgrid(azimuth_bins[:-1], elevation_bins[:-1])

    # 3D surface plot
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    surf = ax1.plot_surface(X, Y, error_grid, cmap='RdYlBu_r',
                            linewidth=0, antialiased=True, alpha=0.9)
    ax1.set_xlabel('True Azimuth (deg)', fontsize=10)
    ax1.set_ylabel('True Elevation (deg)', fontsize=10)
    ax1.set_zlabel('Elevation Error (deg)', fontsize=10)
    ax1.set_title('Elevation Error 3D Surface (Spherical)', fontsize=12, fontweight='bold')
    fig.colorbar(surf, ax=ax1, shrink=0.5)

    # Contour plot
    ax2 = fig.add_subplot(1, 2, 2)
    contour = ax2.contourf(X, Y, error_grid, levels=20, cmap='RdYlBu_r')
    ax2.contour(X, Y, error_grid, levels=10, colors='black', alpha=0.3, linewidths=0.5)
    ax2.set_xlabel('True Azimuth (deg)', fontsize=10)
    ax2.set_ylabel('True Elevation (deg)', fontsize=10)
    ax2.set_title('Elevation Error Contour (Spherical)', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    fig.colorbar(contour, ax=ax2, label='Error (deg)')

    plt.suptitle('Elevation Error Spatial Distribution (Final Spherical Coordinates)',
                 fontsize=14, fontweight='bold')
    plt.tight_layout()

    return fig


def main():
    print("=" * 80)
    print("LIGHTHOUSE V2 BEARING ERROR ANALYSIS (Full 4-Step Process)")
    print("=" * 80)
    print("\nComparing current (buggy) formula vs correct Crazyflie formula")
    print("Showing errors in FINAL spherical coordinates (azimuth, elevation)")
    print("Range: ±80° in azimuth and elevation\n")

    # Generate data
    true_azimuths, true_elevations, azimuth_errors, elevation_errors = generate_error_data()

    # Create plots
    print("\nCreating error analysis plots...")
    fig1 = plot_error_analysis(true_azimuths, true_elevations,
                               azimuth_errors, elevation_errors)

    print("Creating 3D error surface plots...")
    fig2 = plot_3d_error_surface(true_azimuths, true_elevations, elevation_errors)

    # Save figures
    base_path = '/home/gerardo/workspace/lighthouse_ros/attic/lighthouse_angle_calculations'
    output_file1 = f'{base_path}/error_analysis.png'
    output_file2 = f'{base_path}/error_surface_3d.png'

    print("\nSaving plots...")
    fig1.savefig(output_file1, dpi=150, bbox_inches='tight')
    print(f"  Saved: {output_file1}")

    fig2.savefig(output_file2, dpi=150, bbox_inches='tight')
    print(f"  Saved: {output_file2}")

    # Display
    print("\nDisplaying plots...")
    plt.show()

    print("\n" + "=" * 80)
    print("ANALYSIS COMPLETE")
    print("=" * 80)


if __name__ == "__main__":
    main()
