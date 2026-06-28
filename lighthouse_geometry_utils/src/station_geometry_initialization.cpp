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

#include "lighthouse_geometry_utils/station_geometry_initialization.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace lighthouse_geometry_utils
{

StationGeometryInitialization::StationGeometryInitialization()
: next_unified_id_(0) {}

StationGeometryInitialization::~StationGeometryInitialization() {}

void StationGeometryInitialization::reset()
{
  next_unified_id_ = 0;
  station_id_to_unified_id_.clear();
  deck_pose_id_to_unified_id_.clear();
  adjacency_map_.clear();
  first_deck_pose_id_.reset();
}

void StationGeometryInitialization::addSample(
  const std::array<double, 4> & elevations,
  const std::array<double, 4> & azimuths, StationId station_id,
  DeckPoseId deck_pose_id)
{
  // Add station_id to map if not already present
  if (station_id_to_unified_id_.find(station_id) ==
    station_id_to_unified_id_.end())
  {
    station_id_to_unified_id_[station_id] = getUniqueUnifiedId();
  }

  // Add deck_pose_id to map if not already present
  if (deck_pose_id_to_unified_id_.find(deck_pose_id) ==
    deck_pose_id_to_unified_id_.end())
  {
    deck_pose_id_to_unified_id_[deck_pose_id] = getUniqueUnifiedId();
  }

  // Store the first deck pose ID
  if (!first_deck_pose_id_.has_value()) {
    first_deck_pose_id_ = deck_pose_id;
  }

  // Calculate station pose in deck frame
  Sophus::SE3d station_in_deck_pose =
    pnp_solver_.solve(elevations, azimuths);

  // Calculate edge cost
  const double cost = calculateEdgeCost(elevations, azimuths);

  // Add edge from deck to station
  addGraphEdge(
    deck_pose_id_to_unified_id_[deck_pose_id],
    station_id_to_unified_id_[station_id], station_in_deck_pose,
    cost);

  // Add opposite edge from station to deck
  addGraphEdge(
    station_id_to_unified_id_[station_id],
    deck_pose_id_to_unified_id_[deck_pose_id],
    station_in_deck_pose.inverse(), cost);
}

NetworkNodePoses StationGeometryInitialization::solve()
{
  if (!first_deck_pose_id_.has_value()) {
    throw std::runtime_error("No samples have been added");
  }

  const UnifiedId root_id =
    deck_pose_id_to_unified_id_.at(first_deck_pose_id_.value());

  // Cost and pose for each node relative to root
  std::map<UnifiedId, double> best_cost;
  std::map<UnifiedId, Sophus::SE3d> pose_from_root;

  // Priority queue: (cost, node_id)
  using QueueEntry = std::pair<double, UnifiedId>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
    std::greater<QueueEntry>>
  queue;

  // Initialize root node
  best_cost[root_id] = 0.0;
  pose_from_root[root_id] = Sophus::SE3d();  // identity
  queue.push({0.0, root_id});

  while (!queue.empty()) {
    const auto [current_cost, current_id] = queue.top();
    queue.pop();

    // Skip if we already found a better path
    if (current_cost > best_cost[current_id]) {
      continue;
    }

    // Explore neighbors
    auto it = adjacency_map_.find(current_id);
    if (it == adjacency_map_.end()) {
      continue;
    }

    for (const auto & edge : it->second) {
      const double new_cost = current_cost + edge.cost;

      // Update if this is a new node or we found a cheaper path
      auto cost_it = best_cost.find(edge.destination_id);
      if (cost_it == best_cost.end() || new_cost < cost_it->second) {
        best_cost[edge.destination_id] = new_cost;
        pose_from_root[edge.destination_id] =
          pose_from_root[current_id] * edge.pose;
        queue.push({new_cost, edge.destination_id});
      }
    }
  }

  // Check that all nodes were visited
  if (best_cost.size() != next_unified_id_) {
    throw std::runtime_error("Graph is not connected");
  }

  // Build result
  NetworkNodePoses result;
  for (const auto &[station_id, unified_id] : station_id_to_unified_id_) {
    result.station_poses[station_id] = pose_from_root[unified_id];
  }
  for (const auto &[deck_pose_id, unified_id] : deck_pose_id_to_unified_id_) {
    result.deck_poses[deck_pose_id] = pose_from_root[unified_id];
  }

  return result;
}

StationGeometryInitialization::UnifiedId
StationGeometryInitialization::getUniqueUnifiedId()
{
  return next_unified_id_++;
}

double StationGeometryInitialization::calculateEdgeCost(
  const std::array<double, 4> & elevations,
  const std::array<double, 4> & azimuths) const
{
  // Calculate bearing vectors for all four sensors
  std::array<Eigen::Vector3d, 4> bearing_vectors;
  for (std::size_t i = 0; i < 4; ++i) {
    bearing_vectors[i] = calculateBearingVector(elevations[i], azimuths[i]);
  }

  // Calculate angular separation between diagonal sensors
  // Diagonal 1: sensors 0 and 2
  const double diagonal1_separation =
    calculateAngularSeparation(bearing_vectors[0], bearing_vectors[2]);

  // Diagonal 2: sensors 1 and 3
  const double diagonal2_separation =
    calculateAngularSeparation(bearing_vectors[1], bearing_vectors[3]);

  // Return 2*pi minus sum of diagonal separations as cost
  // This ensures cost is always positive and smaller when deck is closer to
  // station and can be expected to produce a more accurate PnP solution.
  return 2.0 * M_PI - (diagonal1_separation + diagonal2_separation);
}

Eigen::Vector3d
StationGeometryInitialization::calculateBearingVector(
  double elevation,
  double azimuth) const
{
  const double cos_elevation = std::cos(elevation);
  return Eigen::Vector3d(
    cos_elevation * std::cos(azimuth),
    cos_elevation * std::sin(azimuth),
    std::sin(elevation));
}

double StationGeometryInitialization::calculateAngularSeparation(
  const Eigen::Vector3d & vector1, const Eigen::Vector3d & vector2) const
{
  return std::acos(std::clamp(vector1.dot(vector2), -1.0, 1.0));
}

void StationGeometryInitialization::addGraphEdge(
  UnifiedId source_id,
  UnifiedId destination_id,
  const Sophus::SE3d & transform,
  double cost)
{
  adjacency_map_[source_id].push_back({destination_id, transform, cost});
}

}  // namespace lighthouse_geometry_utils
