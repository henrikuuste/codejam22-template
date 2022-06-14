#pragma once

#include "common.h"
#include "cost.h"
#include "path.h"
#include <memory>
#include <vector>

namespace pathplanning {
// TODO target path vs target waypoint
// NOTE path based would need a next() method or something
// NOTE waypoint based becomes interesting with the space between - from -> to
struct ITargetCriteria {
  using Fitness              = float; // TODO should this be == Cost
  virtual ~ITargetCriteria() = default;

  // Functions that must be implemented by derived class
  virtual Fitness fitness(Path const &path) const                      = 0;
  virtual ITargetCriteria &setAllowedError(StateDistance const &error) = 0;

  virtual Cost heuristic(Path const &path) const { return {}; };
  virtual bool satisfiesCriteria(Path const &path) const { return fitness(path) > 0.f; };
};

using TargetList = std::vector<std::unique_ptr<ITargetCriteria>>;

// TODO target criteria implementations
struct PointTarget : public ITargetCriteria {
  PointTarget(Location target) : goal(target) {}

  Fitness fitness(Path const &path) const override {
    if (path.empty())
      return 0.f;
    Vec2 diff = goal - path.back().loc;
    return diff.x() <= allowed_error.loc_distance.x() and
           diff.y() <= allowed_error.loc_distance.y();
  }

  ITargetCriteria &setAllowedError(StateDistance const &error) override {
    allowed_error = error;
    return *this;
  }

  Cost heuristic(Path const &path) const {
    if (path.empty())
      return Cost::UNKNOWN;
    return distanceBetween(path.back().loc, goal);
  };

  StateDistance allowed_error;
  Location goal;
}
// TargetWithRadius
// PathSegmentTarget | WaypointTarget

} // namespace pathplanning