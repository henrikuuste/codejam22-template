#pragma once

#include "common.h"
#include "cost.h"
#include "state.h"

namespace pathplanning {
struct IVehicle {
  virtual ~IVehicle() = default;

  // TODO: Figure out complete list of required paramters
  struct Constraints {
    Real max_speed;
    Real lin_acc;
    Real ang_acc;
    Vec3 dimensions;
  };


  virtual Cost::CostValue energyCost(State const &from, State const &to) const = 0;
};
} // namespace pathplanning