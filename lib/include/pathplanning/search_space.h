#pragma once

#include "internal/common.h"
#include "state.h"
// #include "target.h"

namespace pathplanning {
struct SearchSpace {
  StateBounds bounds() const;
  enum PARAM_TYPE { CONTINOUS, DISCRETE, BINARY };
};
} // namespace pathplanning