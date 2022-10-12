#pragma once

#include "internal/common.h"
#include "state.h"
#include "search_space.h"

namespace pathplanning {
struct Cost {
  using CostValue = Real;
  enum Classifier { UNKNOWN, FREE, NORMAL, IMPASSABLE };
  // TODO implementation
  // Implicit conversions?
  // Hold invariants - unknown, free and impassable. negative cost
  // Math operators
  // Comparisons

  Cost();
  Cost(CostValue const &value) : value_(value) {}

  Cost &operator+=(Cost const &other);
  Cost &operator+=(CostValue const &other);
  auto operator<=>(Cost const &other);

  friend Cost operator+(Cost const &lhs, Cost const &rhs);
  friend Cost operator+(Cost const &lhs, CostValue const &rhs);
  friend std::ostream &operator<<(std::ostream &out, const Cost &c) {
    out << "Cost: " << c.value_ << "\n";
    return out;
  }

private:
  Classifier type_;
  CostValue value_;
};

struct ICostProvider {
  enum Error { INTERNAL_ERROR, INVALID_STATE_INPUT };
  using CostOrError = expected<Cost, Error>;

  virtual ~ICostProvider()                                        = default;
  virtual CostOrError costBetween(StateQuery const &query)        = 0;
  virtual CostOrError costOfStateChange(StateQuery const &query)  = 0;
  virtual CostOrError costOfEnvTraversal(StateQuery const &query) = 0;
  virtual CostOrError costOfTerrain(StateQuery const &query)      = 0;
  virtual StateBounds bounds() const                              = 0;
  virtual SearchSpace const &searchSpace() const                  = 0;
  // virtual Lock &lock()                                                       = 0;
  // virtual void release(Lock &lock)                                           = 0;
};
} // namespace pathplanning