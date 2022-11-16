#pragma once

#include "internal/common.h"
#include "magic_enum.hpp"
#include "search_space.h"
#include "state.h"

namespace pathplanning {
using namespace magic_enum::ostream_operators;
struct Cost {
  using CostValue = Real;
  enum Classifier { UNKNOWN, FREE, NORMAL, IMPASSABLE };
  // TODO implementation
  // Implicit conversions?
  // Hold invariants - unknown, free and impassable. negative cost
  // Math operators
  // Comparisons

  Cost() = default;
  explicit Cost(CostValue const &value) : value_(value), type_(NORMAL) {}
  Cost(CostValue const &value, Classifier type) : type_(type), value_(value) {}

  Cost &operator+=(Cost const &other);
  Cost &operator+=(CostValue const &other);
  // Should these also take type_ into account?
  bool operator<(Cost const &other) const { return value_ < other.value_; }
  bool operator>(Cost const &other) const { return value_ > other.value_; }
  bool operator==(Cost const &other) const { return value_ == other.value_; }

  friend Cost operator+(Cost const &lhs, Cost const &rhs) {
    // TODO how to add types?
    Cost c;
    c.type_ = lhs.type_;

    c.value_ = lhs.value_ + rhs.value_;

    return c;
  }
  friend Cost operator+(Cost const &lhs, CostValue const &rhs) {
    Cost c;
    c.type_ = lhs.type_;

    c.value_ = lhs.value_ + rhs;

    return c;
  }
  friend std::ostream &operator<<(std::ostream &out, const Cost &c) {
    out << "Cost: " << c.value_ << " " << c.type_ << "\n";
    return out;
  }

  Classifier type() const { return type_; }
  CostValue value() const { return value_; }

private:
  Classifier type_ = NORMAL;
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