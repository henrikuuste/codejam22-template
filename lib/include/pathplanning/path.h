#pragma once

#include "common.h"
#include "state.h"
#include <vector>

namespace pathplanning {
struct Waypoint {
  Location loc;
};

struct Path {
  // TODO interface definition
  using WaypointList = std::vector<Waypoint>;
  WaypointList path;
  bool empty() const {return path.empty();};
  Waypoint back() const {return path.back();};
};
} // namespace pathplanning