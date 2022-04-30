---
id: ddfrbcp2no92t5w7femv5qs
title: Access
desc: ''
updated: 1651289399223
created: 1650539218204
---
## Overview
API for quering [[api.cost]] from environmental knowledge and state change knowledge.

## Requirements and use cases
* Query cost between two adjecent locations/states
* Query cost between two nearby locations/states
* Cost provider/source may have a different discretization (or none at all) from the planner
* Query the total cost of state change in a given envrionment
* #option Query the cost of state change separately
* #option Query the cost of environment traversal separately
* Cost return by the same input, does not change during a single planning session

## Design options
#### Just provide a static map
```cpp
struct CostProvider {
  using CostMap = char*;
  // or
  using CostMap = std::vector<std::vector<Cost>>;
  // or
  using CostMap = Matrix<Cost, Dynamic, Dynamic>;
  
  CostMap map;
  Bounds2D coordinateBounds;
  meter_t step;

  Index loc2Index(Location loc) const;
  Location index2Loc(Index idx) const;
};

Index currentLoc;
map[currentLoc + (-1, 0)] ...
span = map.span(currentLoc + (-1, -1), currentLoc + (1, 1));
span[0, 0] ...
cost = span[0, 0] + calculated(from, to)
```

```cpp
struct CostEdge {
  CostNode* incoming;
  CostNode* outgoing;
  Cost value;
  meter_t distance;
};

struct CostNode {
  std::vector<CostNode*> neighbors;
  // or
  std::vector<CostEdge*> neighbors;
  // or
  std::array<CostEdge*, 8> neighbors;
  Cost value;
  Location loc;
};

struct CostMap {
  std::vector<CostNode> nodes;
};
```

### Just provide a dynamic map
#out-of-scope

#### Struct with functions
```cpp
struct ICostProvider {
  virtual ~ICostProvider() = default;
  virtual CostOrError costBetween(State const &from, State const &to) = 0;
  virtual CostOrError costOfStateChange(State const &from, State const &to) = 0;
  virtual CostOrError costOfEnvTraversal(State const &from, State const &to) = 0;
  virtual StateBounds bounds() const = 0;
  virtual SearchSpace const& searchSpace() const = 0;
  virtual Lock& lock() = 0;
  virtual void release(Lock &lock) = 0;
};

struct ICostCreator {
  // Other interface  
};

struct ICostNotifier {
  hasCostChanged(time, Path const &path)...
};
```

## Note
* Cost out of Bounds
  * Return error
  * **Return unknown cost**
  * Return impassable cost
* Give an option to the user to find out what the bounds are
* Error
  * Invalid state
    * **Return impassable**
  * InternalError - something went wrong internally, we do not know what

