#pragma once

#include "common.h"
#include "state.h"
#include <vector>

namespace pathplanning {
struct Waypoint {};

struct Path {
  // TODO interface definition
  using WaypointList = std::vector<Waypoint>;
};
} // namespace pathplanning