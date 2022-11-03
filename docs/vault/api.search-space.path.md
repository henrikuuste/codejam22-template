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
- The path must consist of a set of positions
- The path must include state parameters that can't be derived from geometric properties
    - generator state
    - payload state
    - sensor states (if something needs to be turned on/off)
    - lights

- 
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

