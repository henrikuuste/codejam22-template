#pragma once

#include "internal/common.h"
#include "state.h"

namespace pathplanning {
struct SearchSpace {
  // TODO another complex one in the general case, but should be simple for CodeJam
  // TODO what do we need?
  State::Bounds bounds() const;
};
} // namespace pathplanning