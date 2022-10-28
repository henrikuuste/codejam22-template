#pragma once

#include "internal/common.h"

namespace pathplanning {

using Location            = Vec2;
using Elevation           = Real; // TODO use Vec<1> instead?
using Orientation         = Vec3; // RPY
using LinearSpeed         = Real;
using AngularSpeed        = Real;
using Acceleration        = Real;
using AngularAcceleration = Real;
using Dimensions          = Vec3;

struct StateDifference;

// TODO defaults?
struct State {
  // TODO coordinate system?
  enum StateField { LOCATION, ELEVATION, ORIENTATION, LIN_SPEED, ANG_SPEED };
  static constexpr size_t StateDim = Location::RowsAtCompileTime + 6; // TODO think about this

  struct Distance {
    // TODO
    Vec2 loc_distance;
  };

  using StateVector = Vec<StateDim>;
  // NOTE essentially moving towards IndexedVector from ukf++
  // Is this really necessary in this codebase to do state access really really fast
  Location loc() const {
    Location _loc = data_.head(2);
    return _loc;
  }
  // TODO direction
  Elevation elevation() const;
  Orientation orient() const; // just heading
  Real heading() const;
  LinearSpeed linear_speed;
  AngularSpeed angular_speed;
  TimePoint time;
  Acceleration acceleration;
  AngularAcceleration angular_acceleration;
  Dimensions dimensions;

  State &operator+=(StateDifference const &other);
  State &operator-=(StateDifference const &other);

  friend State operator+(State const &lhs, StateDifference const &rhs);
  friend State operator-(State const &lhs, StateDifference const &rhs);
  friend StateDifference operator-(State const &lhs, State const &rhs);

  // TODO fix shitty getters and setters
  StateVector getState() const { return data_; }
  void setStateElement(Real value, Index idx) { data_(idx) = value; }
  void loc(Location loc) {
    data_(0) = loc.x();
    data_(1) = loc.y();
  }

private:
  StateVector data_;
  // TODO orient periodicity
  // TODO implementation
};

struct StateDifference {
  Vec2 loc_difference;
  Real orient_difference;
  Real linear_speed_difference;
  Real angular_speed_difference;
  TimeDiff time_difference;

  // TODO orient periodicity - do we want the shortest difference?
  // TODO implementation
};

// making compiler happy
struct StateQuery {
  State from;
  State to;
  StateQuery(State const &from_, State const &to_) : from(from_), to(to_){};
  friend std::ostream &operator<<(std::ostream &out, const StateQuery &q) {
    out << "Querying from:  " << q.from.loc().x() << " to " << q.to.loc() << "\n";
    return out;
  }
};

struct StateBounds {
  StateBounds() = default;
  StateBounds(State const &min, State const &max) : _min(min), _max(max){};
  StateBounds(State const &center, StateDifference const &halfSize);
  State const &min() const { return _min; }
  State const &max() const { return _max; }
  State center() const;
  StateDifference fullSize() const;
  StateDifference halfSize() const;
  // TODO implementation
private:
  State _min;
  State _max;
};

// dont understand use case for this
struct StateRadius {
  State const &center() const;
  StateDifference const &radius() const;
  // TODO implementation
};

// dont understand use case for this
// TODO CostState
struct CostState {
  State state;
  bool payload;
};
} // namespace pathplanning