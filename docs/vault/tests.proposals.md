---
id: heevnidl1pazyyv7m5o7mx8
title: Proposed tests
desc: ''
updated: 1650078824190
created: 1650078071145
---

## 1.
- General unit tests.
- Waypoint management test, are the waypoints updated properly
- In case of different planners, do all of them work on a simple costmap
- Do they give an output?
- Is the output valid (check that it doesn't go through no-go zones, etc)
- Do they work with 1 to n waypoints?

## 2.
 * TEST1 : Start point within the range of the map
 * TEST2 : End point within the range of the map
 * TEST3 : Map is null
 * TEST4 : Start point is null
 * TEST5 : End point is null
 * TEST6 : Obstacle threshhold is too small
 * TEST7 : Obstacle threshhold is too big
 * TEST8 : maxPathLength is too small
 * TEST8 : maxPathLength is too big
 * TEST8 : maxPathLength is moderately small cancelAtMaximum = false
 * TEST8 : maxPathLength is moderately small cancelAtMaximum = true
 * TEST8 : maxPathLength is moderately small cancelAtMaximum = false
 * TEST8 : maxPathLength is moderately big cancelAtMaximum = true
 
## 3.
No tests provided

## 4.
- Check if proposed plan does not return a path when no path is availavable.(only lethal cost in between two waypoints)
- Path returned is actually with shortest time with set speed
- Returned path is shortest possible length
- Path can not hit object with given machine dimensions

## 5.
- Path from start to goal on the free map (simplest case). -> should give nearly straight line output.
- Path from start to goal where obstacle is in the way (in between) so that it would have one logical shortest path.
- Path from start to goal where obstacle is in the way and shortest path requires moving away from the goal. Should have one logical optimal solution.
- Example where starting location is restricted with occupied area around it.
- Example where goal location is restricted with occupied area around it.
- Example where goal location is in the occupied area.
- Simple examples where goal location is oriented in each quarter (0-90, 90-180, 180 - 270, 270-360 degrees) from starting location.
- Complex maze example where path would need to move in each direction to find optimal path to the goal.
- Example where starting location is not on the map.
- Example where goal location is not on the map.
- Example where goal location equals starting location.

## 6.
### sanity
- error handling with 0 or 1 goalpoints
- error handling when no path exists
- sane input -> sane output (no crashes)

### planning ability
- basic straight path
- basic straight path with N obstacles in the way (up to a labyrinth)
- give N goals, record waypoints, erase first goal, re-plan with N-1 goals, compare N-2 waypoints, repeat (consistency is expected unless turning radius is an issue).
- multiple acceptable paths where one is slightly better (optimization)

### if planner is capable of distinguishing these scenarios
- travelling over and under a bridge
- orientation of the vehicle is important for traversal
- direction of travelling is important for cost

## 7.
* Test a known shortest path between two points. (depends if planner is expected to be optimal, or approximately optimal)
* Find a path to goal through an unknown area
* Find a path to a goal that cannot be reached
* Find a path to a goal with multiple shortest paths (depends if returning a path or all shortest and whether planner must be optimal)
* Find a path to goal, which can only be reached by changing vehicle configuration (rotating, payload state/parameters)
* Find a path to a goal, which can be reached by a vehicle with the exact size (planner should not assume vehicle to be larger than it is)
* Covariance/Area - Depending on implementation/interface, the planner might be planning a path to an area (might be necessary), or to a point with covariance (not a good idea on first thought, but not impossible). Opens up a lot of new test cases.

## 8.
```cpp
SCENARIO("Empty input") {
  WHEN("Input waypoint list is empty") {
    THEN("Planner will output empty plan and report an error") { 
      // ...
    }
  }
}

SCENARIO("Target very far away from current position") {
  WHEN("Input WP further away then X metres") {
    THEN("Error is reported with the waypoint and current position") { 
      // ...
    }
  }
}

SCENARIO("Optimal path finding") {
  GIVEN("Costmap X and a near optimal solution Y") {
    WHEN("Planner calculates the optimal path on the costmap X") {
      THEN("Both optimal paths are close to each other") {
        // ...
      }
    }
  }
}
SCENARIO("Idempotency") {
  WHEN("Running planner on an already existing plan") {
    THEN("Generated plan should be close to idenfitcal to the original plan") { 
      // ...
    }
  }
} // If we do have randomness in the planner, then it should be "close to idempotent"
```