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

#ifndef LIGHTHOUSE_GEOMETRY_UTILS__STATION_GEOMETRY_INITIALIZATION_HPP_
#define LIGHTHOUSE_GEOMETRY_UTILS__STATION_GEOMETRY_INITIALIZATION_HPP_

#include "lighthouse_geometry_utils/station_pose_pnp_solver.hpp"

#include <Eigen/Core>

#include <array>
#include <map>
#include <optional>
#include <queue>
#include <vector>

#include "lighthouse_geometry_utils/datatypes.hpp"
#include <sophus/se3.hpp>

namespace lighthouse_geometry_utils
{

/**
 * @brief Initializes Lighthouse base station geometry.
 *
 * Provides functionality for initializing base station poses for subsequent
 * optimization.
 */
class StationGeometryInitialization
{
public:
  /// Constructs a station geometry initializer.
  StationGeometryInitialization();

  /// Destructor.
  ~StationGeometryInitialization();

  /**
   * @brief Adds a measurement sample for initialization.
   *
   * Samples with the same deck_pose_id are considered to be taken from the
   * same deck location, possibly observing different base stations.
   *
   * @param elevations Array of 4 elevation angles in radians (angle above
   * horizontal).
   * @param azimuths Array of 4 azimuth angles in radians (horizontal angle).
   * @param station_id Identifier of the base station that produced this
   * measurement.
   * @param deck_pose_id Identifier for the deck pose where this sample was
   * taken.
   */
  void addSample(
    const std::array<double, 4> & elevations,
    const std::array<double, 4> & azimuths, StationId station_id,
    DeckPoseId deck_pose_id);

  /**
   * @brief Resets the initializer to its default-constructed state.
   *
   * Clears all accumulated samples and internal state, allowing the object to
   * be reused for a new initialization run.
   */
  void reset();

  /**
   * @brief Solves for initial base station poses using collected samples.
   * @return NetworkNodePoses containing estimated poses for all stations and decks.
   */
  NetworkNodePoses solve();

private:
  using UnifiedId = std::size_t;

  /// Edge data containing a destination, pose transformation, and associated
  /// cost.
  struct EdgeData
  {
    /// Unified ID of the destination node.
    UnifiedId destination_id;
    /// SE3 transformation from source to destination.
    Sophus::SE3d pose;
    /// Heuristic cost associated with this edge.
    double cost;
  };

  /// Returns a unique unified ID by incrementing internal counter.
  UnifiedId getUniqueUnifiedId();

  /**
   * @brief Calculates the heuristic cost associated with a measurement.
   *
   * @param elevations Array of 4 elevation angles in radians.
   * @param azimuths Array of 4 azimuth angles in radians.
   * @return The calculated edge cost.
   */
  double calculateEdgeCost(
    const std::array<double, 4> & elevations,
    const std::array<double, 4> & azimuths) const;

  /**
   * @brief Calculates a normalized bearing vector from elevation and azimuth.
   *
   * @param elevation Elevation angle in radians (angle above horizontal).
   * @param azimuth Azimuth angle in radians (horizontal angle).
   * @return Normalized 3D bearing vector.
   */
  Eigen::Vector3d calculateBearingVector(
    double elevation,
    double azimuth) const;

  /**
   * @brief Calculates angular separation between two unit-length vectors.
   *
   * @param vector1 First normalized vector.
   * @param vector2 Second normalized vector.
   * @return Angular distance in radians (arccos of dot product).
   */
  double calculateAngularSeparation(
    const Eigen::Vector3d & vector1,
    const Eigen::Vector3d & vector2) const;

  /**
   * @brief Adds an edge to the pose graph.
   *
   * @param source_id Unified ID of the source node.
   * @param destination_id Unified ID of the destination node.
   * @param transform SE3 transformation from source to destination.
   * @param cost Cost associated with this edge.
   */
  void addGraphEdge(
    UnifiedId source_id, UnifiedId destination_id,
    const Sophus::SE3d & transform, double cost);

  /// PnP solver for calculating station poses.
  StationPosePnPSolver pnp_solver_;

  /// Counter for generating unique unified IDs.
  UnifiedId next_unified_id_;

  /// Map from station ID to unified ID.
  std::map<StationId, UnifiedId> station_id_to_unified_id_;

  /// Map from deck pose ID to unified ID.
  std::map<DeckPoseId, UnifiedId> deck_pose_id_to_unified_id_;

  /// Adjacency map: source unified ID -> vector of edges.
  std::map<UnifiedId, std::vector<EdgeData>> adjacency_map_;

  /// The first deck pose ID recorded.
  std::optional<DeckPoseId> first_deck_pose_id_;
};

}  // namespace lighthouse_geometry_utils

#endif  // LIGHTHOUSE_GEOMETRY_UTILS__STATION_GEOMETRY_INITIALIZATION_HPP_
