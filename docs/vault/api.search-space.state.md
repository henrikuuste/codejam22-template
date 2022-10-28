---
id: b289ilq1gtdoj48qr6ghgg4
title: State
desc: ''
updated: 1651309018163
created: 1650545807481
---
## Overview
What is state? How is it specified?

## Requirements and use cases
- Must include all relevant non-static parameters
  - Location
  - Linear/angular velocity
  - Orientation
  - Payload/generator state
  - Battery level
- State space >= search space
- State parameters must be individually accessible
- Must be able to configure which state parameters shall be used
  - User is notified if a non-configured parameter is accessed
- Must be able to get difference between two states

## Design options
#### Just a struct
```cpp
struct State {
  Location loc; // 2D
  Orientation orient; // just heading
  LinearSpeed linear_speed;
  AngularSpeed angular_speed;

  Vec<7> vec() const;
};

struct CostState : public State {
  bool payload;
};

struct StateControl {
  // TODO
};
```

#### Vectors
```cpp
struct State {
  Location loc() const;
  Orientation orient() const;
  LinearSpeed linear_speed() const;
  AngularSpeed angular_speed() const;

  Vec<7> vec() const;

private:
  Vec<7> data_;
};
```
