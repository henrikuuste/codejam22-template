# Domain and usage
* UGV (not UAV)
* Path planner will be invoked by the vehicle in real-time
  * At multiple resolution levels/scales (soft real-time)
  * Some of the small scale invocations will have hard real-time constraints
* Path planner will be invoked by a mission solver
  * On C2, simulation or on the vehicle
  * Probably invoked 1000+ times per solver run

# Notes

# Path planner
* Search algorithm
  * Given
    * Search space (range)
    * Evaluation criteria
  * Find
    * One or more targets matching the criteria
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

## Search space
* Usually called configuration space
* Vehicle can be in different configuration - aka State
* Space should contain all the dimensions of the state that we are optimizing for (aka can control)
  * Spatial
    * Position
    * Orientation
    * Velocity

* UGV can have payloads
  * Payload state can affect the cost of traversing an area (going under bridge with an extended payload)
  * We may have control over state of the payload
* UGV can rotate (change heading)
  * Cost can depend on orientation

## Constraints
Depend on
  * Vehicle state
    * Direction of movement - which side of the road are we driving on, or are we going up or down a slope
  * Location in configuration space

```C++
Cost queryCost(Location, State);
```

## Target criteria
Depends on
  * Location in configuration space
    * May still mean that we have different orientations or payload states as a target
  * Location may and probably should include some uncertainty
    * Area around the point - possible configurations that are considered valid
    * Fitness of the path may be affected by how close we get to the desired State
    * Uncertainty will come from multiple sources
      * User specified uncertainty
      * User error (finger on a zoomed out map)
      * Map error
      * Known errors of vehicle motion and localization

## Heuristic
Depends on
  * Location in configuration space
  * Target criteria

## Configurability
* Time limit
* Maximum number of paths
* Diagnostics
* Error handling