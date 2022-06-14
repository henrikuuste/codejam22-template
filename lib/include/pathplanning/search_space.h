#pragma once

#include "common.h"
#include "state.h"
#include "target.h"

namespace pathplanning {
struct SearchSpace {
  StateBounds bounds() const;
};
} // namespace pathplanning