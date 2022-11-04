Study materials:
https://motion.cs.illinois.edu/RoboticSystems/

 Questions:
1. (Mainly about semantics and how to interpret the output of the planner)  
Classically, path refers to a set of positions in which the robot should be while exectuting the motion. It does not tell anything about how it should be traversed (what are the velocities, orientations, other state parameters such as generator etc). Furthermore, classically the path is used to generate the trajectory which is given to the controllers as a target. Trajectory adds the "how the path is traversed part" (classically velocity profile), given kinematic constraints, and is continuous. Clearly this simplistic definition of a path does not satisfy our needs. If the trajectory is generated from a path, then it's not possible for the trajectory generator to know if the user wanted the robot to traverse some part of the path with some given velocity. Or, if it had to turn the generator on at some point. How should we think about a path? Is it still a path, but with extras that we should be using? Can we generate trajectories from those "paths with extras"? Would those still be called trajectories? Should we use some other term instead of a path (what term)? Is it something else rather than a path that our planner should output? Should we rename our planner from "path planner" to something else?  
**a. Henri**: Generally the output of the planner should be at least some sort of ordered set of control space data - meaning, which parameters and how should you change in order to get to this optimal path. This does not mean all parameters are controlled by the planner - for example the planner can only really set the desired/max speed, but the controller is free to alter that based on actual conditions and kinematics. If the vehicle model is the same for the cost calculation and the controller, then you just hope that it is close enough. But you could sacrifice that in order to get a faster computation time. So that is the minimal output - otherwise it is useless. You can then of course also output a bunch of diagnostic data - estimated cost, estimated state of the vehicle at different points etc, but that is mainly there for analytic purposes. In terms of the control space output - it should in general follow the idea of least constraint - meaning that if you have a straight line with the same speed, you would not give a new control space state for every 1 m of that line. Same applies to any other dimension of the state - we want to record changes and we want to constrain it only as much as is needed for the next level of planning or control. In the end that is up to the specific planner implementation and not part of the API though, so it can only be a documented rule.


Terminology horror:

* Spaces 
    * State space - All possible states of all the parameters of the system
        * Position
        * Orientation
        * Linear/angular velocity
        * Acceleration
        * Battery level
        * Payload stuff
        * Generator/lights/etc
    * Configuration space - All states that describe position in physical world. All possible poses (given robot geometry and constraints) in which the robot can exist in the world, including the ones that collide with an obstacle.
        * Includes position and/or rotation, but not whether lights are on/off
        * Payload position
    * Control space - All possible states of controllable parameters.
        * The parameters of this space are directly used by the robot's controllers
    * Search space - States of parameters the search is conducted in
    * Task space - Set of parameters that are relevant for the task.
        * Abstraction layer for the parameters that are directly involved in solving the given task.
        * For example drawing with a marker on a board, task space is 2D space (board) and does not care of marker movement above the board
        * Might not be relevant for path planner, but could be useful for mission planner
        * Task space does not depend on the robot
    * Free space - Subset of configuration space in which the robot is not occupied by obstacles.
    * Physical space - Describes the environment
    * Workspace - Defines the set of parameters in which the "tool" for the task can be.
        * E.g. Autonomous snow plowing: in what configurations can the plow physically be in the environment.
        * E.g. In what configurations a autonomous surveillance robot's camera can be (where can it physically see) 
    * Cost space - State space abstraction into a vector space that describes the penalties of state changes.

* Planners
    * Motion planner - generates a representation of the robot's motion
    * Path planner
    * Trajectory planning/generation

* Motion 
    * The change of state at any instant in time of a body (or bodies). [2]
    * the planned motion should follow the path or follow that path closely [3]
    * the current motion can influence the path planning. If driving in one direction at a high speed, a sudden change in velocity might not be the best idea even if that would be necessary to get on the shortest path. Or the starting pose of a nonholonomic robot, which influences the possible path significantly. [3]
    * motion is related to time (change of position over time), but the path itself is not. Even if the motion goes along the path exactly it could have various trajectories for velocity and/or acceleration. Your robot could stop for example to let another robot cross its path, which is a change in motion but not the path itself. [3]
    * motion can be influenced by other things, like wind for example [3]
    * The motion will be executed by the controller [1]
    * When talking about motion planning, it is planning motions for robots to move from point A to point B (such as for mobile robots, etc.) or pose A to pose B (such as for manipulators, etc.). In order to do so, a number of constraints need to be taken into account: collision avoidance, joint limits, velocity/acceleration limits, jerk limits, dynamic balance, torque bounds, and many more. In this sense, not only the the robots is considered but also its environment (e.g., to avoid collision, to how remain balanced). Considering this, motion planning is kind of trajectory generation with lots of constraints. [2]

* Path
    * A path implies that a motion happens along it and a motion necessarily happens along a path [3]
    * The position of a body or bodies over a period of time without worrying about velocity or higher order terms. [2]

* Trajectory
    * The state of a body or bodies over a period of time. [2]
    * A path is not equal to a trajectory. A trajectory is a path and information of how to traverse the path with respect to time, a.k.a a velocity profile [2]


References:

1. http://motion.cs.illinois.edu/RoboticSystems/WhatIsMotionPlanning.html
2. https://robotics.stackexchange.com/questions/6683/what-is-the-difference-between-motion-planning-and-trajectory-generation#:~:text=Motion%20%2D%20The%20change%20of%20state,over%20a%20period%20of%20time.
3. https://robotics.stackexchange.com/questions/8302/what-is-the-difference-between-path-planning-and-motion-planning#:~:text=Path%20planning%20is%20the%20process,follow%20the%20path%20you%20planned.
4. 