#pragma once

#include "internal/common.h"
#include "state.h"
#include <vector>

namespace pathplanning {
/*
// Why is this struct necessary? Do we want to keep some extra information here?
struct Waypoint {
  // TODO is target a good name?
  State target; // what defines a waypoint
};
*/

struct Path {
  // TODO interface definition
  std::vector<State> path;
  std::vector<StateBounds> bounds;
  // vector<OptimizationMask> mask;

  Real cost;
  float fitness;

  bool empty() const { return path.empty(); };
  State back() const { return path.back(); };

  void emplace_back(State point) { path.emplace_back(point); }
};
} // namespace pathplanning