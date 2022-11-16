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
struct State { // version 1 simplification
  Location loc; // 2D
  Orientation orient; // orientation
  LinearSpeed linear_speed; // lets not use for now
  AngularSpeed angular_speed; // lets not use for now

  Vec<7> vec() const;
};

// wtf?
struct CostState : public State {
  bool payload; // let's say it's outside of the scope for v1 and hackathon
};

struct StateControl {
  // TODO
};
// end wtf?
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
