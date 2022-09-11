#pragma once

#include "internal/common.h"
// TODO: Make agv_math into a library and include here (currently units uses "math" namespace)

namespace agv_math {
template <class T> pathplanning::Real distanceBetween(T vec1, T vec2) { return (vec1 - vec2).norm(); }
} // namespace agv_math