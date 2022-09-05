#pragma once

#include "cost.h"
#include "internal/common.h"
#include "search_space.h"
#include "state.h"

namespace pathplanning {
struct ICostProvider {
  enum Error { INTERNAL_ERROR, INVALID_STATE_INPUT };
  using CostOrError = expected<Cost, Error>;

  virtual ~ICostProvider()                                                   = default;
  virtual CostOrError costBetween(State const &from, State const &to)        = 0;
  virtual CostOrError costOfStateChange(State const &from, State const &to)  = 0;
  virtual CostOrError costOfEnvTraversal(State const &from, State const &to) = 0;
  virtual CostOrError costOfTerrain(State const &from, State const &to)      = 0;
  virtual StateBounds bounds() const                                         = 0;
  virtual SearchSpace const &searchSpace() const                             = 0;
  virtual Lock &lock()                                                       = 0;
  virtual void release(Lock &lock)                                           = 0;
};
} // namespace pathplanning