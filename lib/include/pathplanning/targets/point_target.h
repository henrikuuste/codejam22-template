#pragma once

#include "../target.h"

namespace pathplanner {

/**
 * @brief Exact location that must be reached within the allowed numeric error.
 *
 */
struct PointTarget : public ITargetCriteria {
  PointTarget(Location target) : goal(target) {}

  Fitness fitness(Path const &path) const override {
    if (path.empty())
      return 0.f;
    // TODO think about this
    Vec2 diff = (goal - path.last().loc()).cwiseAbs();
    return (diff.x() <= allowed_error.loc_distance.x()) &&
           (diff.y() <= allowed_error.loc_distance.y());
  }

  ITargetCriteria &setAllowedError(State::Distance const &error) override {
    allowed_error = error;
    return *this;
  }

  Cost heuristic(Path const &path) const {
    if (path.empty())
      return Cost::UNKNOWN;
    return distanceBetween(path.last().loc(), goal);
  };

  StateDistance allowed_error;
  Location goal;
};

} // namespace pathplanner