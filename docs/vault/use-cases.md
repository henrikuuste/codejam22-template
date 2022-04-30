---
id: b70h1rlawsyeoez8exlbhoi
title: Use Cases
desc: ''
updated: 1650545762174
created: 1650543642998
---
## Basic

#### Open flat field with no traversal difficulties
* Ideal straight paths

#### Previous with target (goal) that is an area of some size
* Planner should account for the area
* Given area of equal fitness
  * Plan to the most optimal edge of the area
* Given area with gradient fitness from the center
  * Plan to the center of the area

#### Target area partially inside obstacle
* Could be center or one edge or ...
* Plan to the most optimal part of the area that is not inside the obstacle

#### Target completely inside obstacle
* Return error - unable to find a valid path / path blocked #todo
  * Return path to a point near the obstacle that is closest to target #tbd

#### Initial position inside obstacle


#### Initial position surrounded by obstacles
* Equal to target inside obstacle

#### Path completely obstructed at some other point
* Equal to previous

#### Plan around simple obstacle in a flat area

#### Plan on roads

#### Follow-me on road

#### Drive through difficult terrain

#### Plan for different sized vehicles

#### Dynamic obstacle moves out of the way
* Scenario
  * Intial plan is around the obstacle
  * Obstacle is cleared and there is a better/shorter path
* Solution #1
  * Call the path planner on a regular basis
    * compare results with current path
    * select based some criteria
* Solution #2
  * Knowing that this is a dynamic obstacle
  * We get plans with and without the obstacle
  * Evaluate the alternative path on a regular basis or based on dynamic obstacle event
  * Compare and select based on some criteria
* Solution #3
  * Check if cost near the current path has changed
    * Try replan
    * Compare and select based on some criteria

## Advanced

#### Checkpoint
* Scenario
  * Checkpoint is blocked initially
  * Upon arrival, the vehicle is cleared to move ahead, the road block is removed
* Solution
  * Mission planner
    * Plans a path up to the checkpoint
      * with the end behavior of stop on obstacle
      * plan can be further than the checkpoint, but there is a waypoint at the checkpoint
      * In this case the input to the global path planner would not include an obstruction at the checkpoint (just road cost)
      * Local planner would see the road block and stop there
    * Upon resuming
      * the behavior is disabled
      * a new local path is planned
        * now the path is clear

#### Plan up a slope

#### Plan down a slope

#### Area where generator is not allowed

#### Under a bridge 

#### Over a bridge

#### Multi-storey car park

#### Stuck

#### Gap crossing

#### Change in vehicle dimensions depending on plan

#### Time dependent change in vehicle dimensions

#### Brush cutter movement
* Changing dimensions due to payload motion
* Payload motion can depend on plan

#### Time dependent environmental cost
* Ex. Cold weather during the night - mud freezes
* Ex. Known rotation of guards or changes in checkpoint
* Ex. Bridge raised/lowered

#### Power drain due to use of certain manouvers

#### Unknown area exploration

#### Vehicle intertia and turning radius

#### Pivot turns possible vs impossible

#### Going through a tunnel with a vehicle that is long and narrow

## Complex

#### Road with deep ruts

