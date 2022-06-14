#pragma once

#include "common.h"

namespace pathplanning {

using Location     = Vec2;
using Elevation    = Real; // TODO use Vec<1> instead?
using Orientation  = Real;
using LinearSpeed  = Real;
using AngularSpeed = Real;

struct StateDistance;

// TODO defaults?
struct State {
  // TODO coordinate system?
  enum StateField { LOCATION, ELEVATION, ORIENTATION, LIN_SPEED, ANG_SPEED };
  static constexpr size_t StateDim = Location::RowsAtCompileTime + 4; // TODO think about this

  using StateVector = Vec<StateDim>;
  // NOTE essentially moving towards IndexedVector from ukf++
  Location &loc() const;
  // TODO direction
  Elevation &elevation() const;
  Orientation &orient() const; // just heading
  LinearSpeed linear_speed;
  AngularSpeed angular_speed;
  TimePoint time;

  State &operator+=(StateDistance const &other);
  State &operator-=(StateDistance const &other);

  friend State operator+(State const &lhs, StateDistance const &rhs);
  friend State operator-(State const &lhs, StateDistance const &rhs);
  friend StateDistance operator-(State const &lhs, State const &rhs);

private:
  StateVector data_;
  // TODO orient periodicity
  // TODO implementation
};

struct StateDistance {
  Vec2 loc_distance;
  Real orient_distance;
  Real linear_speed_distance;
  Real angular_speed_distance;
  TimeDiff time_distance;

  // TODO orient periodicity - do we want the shortest distance?
  // TODO implementation
};

struct StateBounds {
  StateBounds(State const &min, State const &max);
  StateBounds(State const &center, StateDistance const &halfSize);
  State const &min() const;
  State const &max() const;
  State center() const;
  StateDistance fullSize() const;
  StateDistance halfSize() const;
  // TODO implementation
};

struct StateRadius {
  State const &center() const;
  StateDistance const &radius() const;
  // TODO implementation
};

// TODO CostState
struct CostState {
  State state;
  bool payload;
};
} // namespace pathplanning