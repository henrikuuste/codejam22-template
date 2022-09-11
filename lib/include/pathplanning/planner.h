#pragma once

#include "cost.h"
#include "internal/common.h"

namespace pathplanning {

struct IPlanner {
  virtual ~IPlanner() = default;

  // TODO think about general error format for ease of use
  enum Error { INVALID_INITIAL_STATE, OUT_OF_BOUNDS, INTERNAL_ERROR };

  using PathOrError = expected<Path, Error>;

  // Functions that must be implemented by derived class
  virtual PathOrError plan(State const &initial, TargetList const &targets) = 0;
  // TODO a more general config input?
  virtual IPlanner *setCostProvider(ICostProvider *provider) = 0;
  virtual IPlanner *setTimeLimit(seconds_t limit)            = 0;
  // TODO what is the diagnostics format?
  virtual PlannerDiagnostics getDiagnostics() = 0;
};

} // namespace pathplanning