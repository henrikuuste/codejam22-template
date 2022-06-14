---
id: 6mbqaqfoesx0daw5fmkz0rg
title: Search Space
desc: ''
updated: 1651370042141
created: 1650545807485
---
## Overview
How do we describe the available search space to the planner?

## Scope
* 2D location
* Orientation
  * Slopes
  * Heading of the vehicle (fitting through gaps, target heading etc)
  * Pivot turn available vs not - vehicle turning limits
* Speed
  * Inertia and braking distance
  * Required speed for slopes
  * Required speed for difficult terrain (bumpy, muddy ...)
  * Speed limits of the vehicle configuration
* Payload that changes vehicle geometry
  * Externally controlled
* #optional Elevation

## Problem(s)
* Input to the planner is about desired states and constraints
  * Specified in search space
  * Ex. Want to reach a certain point in space, with some accuracy or some limit to deviation to the side (corridor)
  * We can also say that we do not care about a specific part of the state
    * Ex. I want to get to this location, don't care about heading
* Output from the planner is about possible states that could fulfil the input criteria
  * Move from point x to y to z ...
  * Move from point .... with speed ...
  * Move from point .... with a maximum deviation of ....
    * Planner -> controller
    * Planner -> planner -> controller
    * Ex - points and polygons (that contain the points)
    * Ex - points, lines and distance
  * Maybe specified in control space?
* Multiple spaces
  * Cost space - used to look up cost
  * Search space - used to run search algorithm.
    * Defines the possible states that are in the input and output
  * Control space - things the search algorithm can control
  * Cost space != Search space != Control space
* In the real world system the search space is highly dynamic
  * Different based on vehicle model and variant
  * Different based on vehicle configuration
  * Different based on autonomy configuration and limits

## Examples
#### 2D location
* Cost - loc, direction
* Search - loc, (direction)
* Control - loc

#### Location + independent payload
* Ex. zone with behavior to deploy payload
* Cost - loc, pl
* Search - loc
* Control - loc

#### Location + speed
* Possible external limits to speed
  * Vehicle capability
  * User input
* Cost - loc, speed
* Search - loc, speed
* Control - loc, speed limits
* Take vehicle inertia into account

#### Location + elevation
* Cost - loc, el
* Search - loc, el
* Control - loc
* Changing location may change elevation
  * Where do we get this information?
    * Control state change -> Search state change

#### Location + orientation
* Cost - loc, direction, r, p, y
* Search - loc, direction, y
* Control - loc, y
* Given a change in position, yaw is limited, roll and pitch are automatic
  * roll and pitch are just environmental cost
* Given a change in yaw, the next change in position is limited
  * Cost of moving sideways is impassable
* Dependency between direction and yaw

## Requirements and use cases
* Specify which dimensions are available
* Specify limits to those dimensions
* Specify how the dimensions can be discretized
* Specify how the dimensions depend on each other

## Design options


