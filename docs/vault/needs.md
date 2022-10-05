## Needs for codejam - in progress

Make an interface - this exists before the codejam

- User needs to know that the interface works
- User needs to find a path and cost between two states in a given search space
  - User needs to receive feedback if there is no viable path
  - User wants to register their planner to the planner hierarchy
- User wants to use the planner in different frames (global, local (for example slam map), relative)
  - For the first prototype, more complex cost providing/merging is out of scope
- User needs to be able to provide a cost provider
  - How to merge costs
- User needs to have an acces to internal diagnostics of the library
  - error, fitness, configuration,
- User needs to be able to specify the timeout for planning
- User needs to be able to access intermediary results of the planning process
- User needs to be able to configure planning behaviour
  - heuristic,
-

Make some visualization and simulation tool?

-

Make an implementation that fits the interface and solves

- simpler problem (2d somethingsomething)
  -
- competes with others to be "better"
  -
