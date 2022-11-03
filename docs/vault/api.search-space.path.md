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
- The path is used to generate the trajectory
- The path has to have position information in it
- The path must include necessary state parameters that can't be derived from geometric properties
  - generator state
  - payload state
  - sensor states (if something needs to be turned on/off)
  - lights

- The path might be invalid if it's not traversed between some velocity limits
  - The path shall include conditions in which it is viable.
## Design options
#### List of waypoints

```cpp


struct Waypoint {
  Location location;

};
```

#### Option 2
```cpp

```

