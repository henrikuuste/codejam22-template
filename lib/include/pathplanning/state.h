#pragma once

#include "internal/common.h"

namespace pathplanning {

using Location     = Vec2;
using Elevation    = Real; // TODO use Vec<1> instead?
using Orientation  = Real;
using LinearSpeed  = Real;
using AngularSpeed = Real;

Real distanceBetween(Location const &a, Location const &b) { return (a - b).norm(); }

struct State {
  // TODO this is one of the more complex bits to figure out
  // TODO should try and keep it simple for the CodeJam
  struct Distance {
    // TODO
  };

  struct Bounds {
    // TODO
  };

  // Interface
  // TODO should state be mutable?
  Location &loc();
  Elevation &elevation();
  Orientation &orient();
  TimePoint &time();

  // Math
  friend State operator+(State const &lhs, Distance const &rhs);
  friend State operator-(State const &lhs, Distance const &rhs);
  friend Distance operator-(State const &lhs, State const &rhs);

private:
  // Data
};

} // namespace pathplanning