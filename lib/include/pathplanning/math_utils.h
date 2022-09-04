#pragma once

#include "common.h"
// #include "state.h"

// TODO: Make agv_math into a library and include here (currently units uses "math" namespace)
using namespace pathplanning; //Don't like it, but need another "common.h in the math library for this"

namespace agv_math{
  template<class T>
  Real distanceBetween(T vec1, T vec2){
    return (vec1 - vec2).norm();
  }
}