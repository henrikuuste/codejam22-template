---
id: evmn73kudpb2nkn7tvych5d
title: Path
desc: ''
updated: 1651289339719
created: 1651289311429
---
## Overview
Output of the path planner.

## Requirements and use cases
- The path is an ordered set of state space data 
- The path might be invalid if it's not traversed between some bounds (e.g. velocity limits)
  - The path shall include conditions in which it is viable (or the planner should return conditions along with the path)
- Should include what parameters/state variables were optimised for.
## Design options
#### Option 1: List of conditioned states 

```cpp

using OptimizationMask = vector<bool>;

struct Path {
  vector<State> path; // contains all state info
  vector<StateBounds> bounds; // conditions/bounds for each state in which the path is still valid
  vector<OptimizationMask> mask; // tells which state variables were optimized for

  Real cost;
  float fitness; // root mean square error of (closest path state - waypoint centroid) for example?
};

```

#### Option 2: List of conditioned state changes
```cpp
using OptimizationMask = vector<bool>;

struct Path {
  State initial_state;
  vector<DeltaState> delta_path;

  vector<StateBounds> bounds; 
  vector<OptimizationMask> mask;

  Real cost;
  float fitness;
};

```

