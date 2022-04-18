---
id: j2634enztfp9u21yzf5u509
title: Planner
desc: ''
updated: 1650276607015
created: 1650273170571
---
## Overview

## Requirements and use cases
* Path planning search
  * Given
    * Multi-dimensional connected search space (eg 2D grid)
    * constraints (eg obstacles, traversability)
    * starting point in search space
    * target criteria (eg goal point in the search space)
    * heuristic (eg. closest distance)
  * Find
    * Contiguous path from starting point to target point in search space
      * Matching constraints
      * Optimizing for the constraints and heuristic
* Given cost access point
* Target criteria can include multiple strictly ordered waypoints
* Given an empty list of waypoints, then return error #todo
* Given a single waypoint, the end of the return path shall match that waypoint
  * Given multiple, then it should match the last waypoint
* Given a waypoint matching the initial state, then return a path with just the initial state
* Given valid initial state and waypoints, the path should start with initial state and end with a state matching the last waypoint
* Given valid waypoints, the patch should contain at least one state for each matching waypoint
* Given an initial state that is outside of search space bounds, then return error #todo
* Given any waypoint that is outside of search space bounds, then return error #todo
* Exceeding the time limit returns error #todo
  * May still return a partial or non-optimal path #tbd
* Exceeding the optional maximum path length returns error #todo
  * May still return a partial or non-optimal path #tbd
  * Consider the cost interface here? #todo
* Additional constraints - corridor or limited search area constraints #out-of-scope
  * Part of cost interface? #todo
* Return multiple/alternate routes? #tbd
  * Equal cost use case

## Design options
#### Stateless
```cpp
Path plan(...);
```
* Limits options

#### Stateful
```cpp
struct Planner {
  Path plan(CostMap const &map, State const &initial, TargetCriteria const &targets);
};

// or

struct Planner {
  void setCostMap(CostMap const &map);
  Path plan(State const &initial, TargetCriteria const &targets);
};
```
* could keep an internal data structure
  * reuse it for multiple calls
  * likely that multiple calls will be in the same area
  * efficiency/performance

#### Error handling
```cpp
struct PlannerError {
  Path path;
  ErrorEnum error;
  ErrorDiagnostics diagnostics;
};

using PathOrError = tl::expected<Path, PlannerError>;

struct Planner {
  void setCostMap(CostMap const &map);
  PathOrError plan(State const &initial, TargetCriteria const &targets);
};

auto result = planner.plan(...);
```
#### Configuration
```cpp
struct Planner {
  void setCostMap(CostMap const &map);
  Planner& setTimeLimit(seconds_t limit);
  Planner& setDistanceLimit(std::optional<meter_t> limit);
  PathOrError plan(State const &initial, TargetCriteria const &targets);
};

planner.setTimeLimit(1_s).setDistanceLimit(2_km);
```

#### Diagnostics
```cpp
struct Path {
  // diagnostics
};

struct Planner {
  PathOrError plan(State const &initial, TargetCriteria const &targets);
};

// or

struct Planner {
  PathOrError plan(State const &initial, TargetCriteria const &targets);
  Diagnostics getDiagnostics() const;
};
```