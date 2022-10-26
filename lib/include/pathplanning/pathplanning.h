#pragma once

#include <pathplanning/export.h>
#include <string_view>

#include "internal/common.h"
#include "cost.h"
#include "search_space.h"
#include "vehicle.h"
#include "path.h"
#include "target.h"
#include "targets/point_target.h"

namespace pathplanning {

PATHPLANNING_EXPORT std::string_view version();

// TODO why is this in both this file and planner.h
struct IPlanner {
  virtual ~IPlanner() = default;

  enum Error { INVALID_INITIAL_STATE, OUT_OF_BOUNDS, INTERNAL_ERROR };

  using PathOrError = expected<Path, Error>;

  // Functions that must be implemented by derived class
  // TODO Question: Why give multiple targets if output is only one path?
  virtual PathOrError plan(State const &initial, TargetList const &targets) = 0;
  virtual IPlanner *setCostProvider(std::weak_ptr<ICostProvider> provider)  = 0;
  virtual IPlanner *setTimeLimit(seconds_t limit)                           = 0;
  // virtual PlannerDiagnostics getDiagnostics()                               = 0;
protected:
  std::weak_ptr<ICostProvider> cost_provider;
};

} // namespace pathplanning