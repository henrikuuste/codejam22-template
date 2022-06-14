#pragma once

#include "common.h"
#include "cost.h"
#include "state.h"

namespace pathplanning {
struct IVehicle {
  virtual ~IVehicle() = default;

  virtual Cost::CostValue energyCost(State const &from, State const &to) const = 0;
};
} // namespace pathplanning