---
id: 2vqtv9mzq33li9u91n1z5wa
title: Cost
desc: ''
updated: 1651291971379
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
### Just use a number
#### Number value only
```cpp
using Cost = float;
```

#### Number + unknown/error
```cpp
using Cost = std::optional<float>
```

#### Number + classifier
```cpp
enum class Classifier { UNKNOWN, FREE, IMPASSABLE };
using Cost = expected<float, Classifier>;
```

#### Number + helper functions
```cpp
using Cost = float;
bool isFree(Cost const &cost);
bool isUnknown(Cost const &cost);
bool isImpassable(Cost const &cost);
```

### With strict units
Same as [[just a number|api.cost#just-use-a-number]] except we limit math.
#### Just use a units library
```cpp
using Cost = units::kilowatt_hour_t;
```

### Structured
#### Structured wrapper
Hide internal implementation

```cpp
struct Cost {
  auto operator<=>(Cost const &other);
  Cost& operator+=(Cost const &other);
  // ...

private:
  units::kilowatt_hour_t cost; // or whatever
};
```

#### Internally separate cost and risk
Same as previous, internally we just use multiple values.
```cpp
struct Cost {
  using CostUnit = expected<units::kilowatt_hour_t, Classifier>;
  using RiskUnit = expected<float, Classifier>;

  CostUnit cost() const;

private:
  CostUnit cost_ = Classifier::UNKNOWN;
  RiskUnit risk_ = Classifier::UNKNOWN;
};
```

#### Vector of cost and risk
```cpp
struct Cost {
  using CostUnit = expected<units::kilowatt_hour_t, Classifier>;
  using RiskUnit = expected<float, Classifier>;

  CostUnit cost = Classifier::UNKNOWN;
  RiskUnit risk = Classifier::UNKNOWN;
};
```