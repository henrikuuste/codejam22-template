#pragma once

#include "internal/common.h"

namespace pathplanning {

// TODO cache path cost?

struct Path {
  // TODO elements
  struct Waypoint {
    // TODO
  };

  using WaypointOrError = expected<Waypoint, PathError>;
  // TODO constraints (corridor)

  // TODO interface
  bool empty() const;
  WaypointOrError last() const;
  WaypointOrError first() const;
  // append
  // remove
};

} // namespace pathplanning