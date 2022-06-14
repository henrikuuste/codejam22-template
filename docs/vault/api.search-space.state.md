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
