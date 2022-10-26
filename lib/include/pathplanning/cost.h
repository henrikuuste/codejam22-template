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

  Cost() = default;
  explicit Cost(CostValue const &value) : value_(value) {}
  Cost(CostValue const &value, Classifier type) : type_(type), value_(value) {}

  Cost &operator+=(Cost const &other);
  Cost &operator+=(CostValue const &other);
  // auto operator<=>(Cost const &other);
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
    out << "Cost:  " << c.value_ << "\n";
    return out;
  }

  Classifier type() const { return type_; }

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