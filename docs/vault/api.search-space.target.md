---
id: r33sn2j2gsbdho1jrrx6n9v
title: Target
desc: ''
updated: 1651370078755
created: 1650545831231
---
## Overview
How do we specify target criteria?

## Requirements and use cases

## Design options
#### Just state
```cpp
using TargetCriteria = State;
```

#### State with constraints
```cpp
struct TargetCriteria {
  State state;
  StateBoundsCircular limits;
};
// or
struct TargetCriteria {
  StateBoundsCircular target;
};

// bounds need to support "unlimited"
// rectangular bounds
struct StateBoundsRect {
  State min() const;
  State max() const;
  State center() const;
};
// circular bounds
struct StateBoundsCircular {
  State center() const;
  StateDistance maxDistance() const;
};
// asymmetric bounds...
```

#### Interface class
```cpp
struct TargetCriteria {
  virtual ~TargetCriteria() = default;
  virtual bool atTarget(State const &state) const { return fitness(state) > 0.f; };
  virtual Fitness fitness(State const &state) const = 0;
  virtual Fitness fitness(Path const &state) const = 0; // TODO
  virtual TargetCriteria& setAllowedError(StateDistance const &error) = 0;
  virtual Cost heuristic(State const &state) const { return {}; };
};

plan(...target) {
  target.setAllowedError(...);
  ...
}
```

Example
```cpp
struct BoundedCriteria : public TargetCriteria {
  Fitness fitness(State const &state) const override {
    // check if inside bounds
  };

  Cost heuristic(State const &state) const override {
    return (bounds.center().loc - state.loc).norm();
  }

  StateBounds bounds;
};
```
