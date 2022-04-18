---
id: 2vqtv9mzq33li9u91n1z5wa
title: Cost
desc: ''
updated: 1650279691868
created: 1650277563224
---
## Overview
Cost is a fundamental type in the system.
Path planning is mostly just cost optimization.

## Requirements and use cases
* Compare cost to select optimal case
* Accumulate cost to other cost
* Negative cost?
  * Would imply that this action would actually regenerate a lost resource
* Cost has to have a semantic meaning
  * time
  * distance
  * energy
  * risk
* Cost classifiers
  * impassable
  * unknown
  * free (map cost)
* Any cost accumulation that includes impassable cost becomes impassable
* Any cost accumulation that includes unknown cost becomes unknown
* Combine with heuristic result
* Calculate cost of state transitions
* Calculate fitness of a path - sum of cost? #tbd

## Design options
#### Option 1
```cpp
using Cost = float;
// or
using Cost = std::optional<float>
```


#### Option 2

```cpp
struct Cost {
  auto operator<=>(Cost const &other);

  units::kilowatt_hour_t cost;
};
```

#### Option 2
```cpp

```
