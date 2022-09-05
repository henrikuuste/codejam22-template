#pragma once

#include "cost.h"
#include "internal/common.h"
#include "state.h"

namespace pathplanning {
struct IVehicle {
  virtual ~IVehicle() = default;

  virtual Cost::CostValue costOfStateChange(State const &from, State const &to) const = 0;
  // TODO is the footprint actually needed?
  virtual Geometry footprint() const = 0;
};
} // namespace pathplanning