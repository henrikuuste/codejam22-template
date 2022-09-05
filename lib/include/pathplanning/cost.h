#pragma once

#include "internal/common.h"

namespace pathplanning {
struct Cost {
  using CostValue = float; // could be energy? Need to test it
  enum Classifier { UNKNOWN, FREE, NORMAL, IMPASSABLE };
  // TODO implementation

  // Constructors
  Cost();
  Cost(Cost const &other);
  Cost(Cost &&other);
  Cost(CostValue const value);
  Cost(Classifier const type);

  // Implicit conversions?

  // Math operators
  // Hold invariants - unknown, free and impassable. negative cost
  Cost &operator+=(Cost const &other);
  Cost &operator+=(CostValue const value);

  friend Cost operator+(Cost const &lhs, Cost const &rhs);
  friend Cost operator+(Cost const &lhs, CostValue const rhs);

  // Comparisons
  auto operator<=>(Cost const &other);

private:
  Classifier type_;
  CostValue value_;
};
} // namespace pathplanning