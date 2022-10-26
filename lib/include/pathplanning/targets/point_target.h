#pragma once

#include "../target.h"
#include "../math_utils.h"
namespace pathplanning {

/**
 * @brief Exact location that must be reached within the allowed numeric error.
 *
 */
struct PointTarget : public ITargetCriteria {
  explicit PointTarget(Location target) : goal(target) {}

  Fitness fitness(Path const &path) const override {
    if (path.empty())
      return 0.f;
    // TODO think about this
    Vec2 diff = (goal - path.back().target.loc()).cwiseAbs();
    return (diff.x() <= allowed_error.loc_distance.x()) &&
           (diff.y() <= allowed_error.loc_distance.y());
  }

  ITargetCriteria &setAllowedError(State::Distance const &error) override {
    allowed_error = error;
    return *this;
  }

  Cost heuristic(Path const &path) const override {
    if (path.empty())
      return Cost(0, Cost::UNKNOWN);
    return Cost(agv_math::distanceBetween(path.back().target.loc(), goal));
  };

  State::Distance allowed_error;

private:
  Location goal;
};

} // namespace pathplanning