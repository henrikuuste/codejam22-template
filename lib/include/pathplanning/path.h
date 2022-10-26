#pragma once

#include "internal/common.h"
#include "state.h"
#include <vector>

namespace pathplanning {
struct Waypoint {
  // TODO is target a good name?
  State target; // what defines a waypoint
};

struct Path {
  // TODO interface definition
  using WaypointList = std::vector<Waypoint>;
  WaypointList path;

  bool empty() const { return path.empty(); };
  Waypoint back() const { return path.back(); };

  // Add function for checking if waypoint on path

  void emplace_back(Waypoint wp) { path.emplace_back(wp); }
};
} // namespace pathplanning