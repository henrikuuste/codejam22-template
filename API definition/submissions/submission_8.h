#pragma once

#include <Eigen/Dense>
#include <vector>

using Vector3    = Eigen::Vector3d;
using Quaternion = Eigen::Quaterniond;

struct Constraints {
  bool enable_orientation_constraint = false;
  Quaternion orientation             = Quaternion::Identity();
  // to-do - think of more constraints
  bool enable_generator_constraint = false;
  bool generator_allowed           = false;
};

struct Waypoint {
  Vector3 pos = Vector3::Zero();
  // Each wp can have constraints
  Constraints constraints;
};

struct Zones {
  // constraints can also be defined for areas
  Constraints constraints;
};

struct Path {
  std::vector<Waypoint> waypoints;
};

struct InitialState {
  Vector3 pos       = Vector3::Zero();
  Quaternion orient = Quaternion::Identity();
};

struct CostMap {
  // Will have multiple formats here, but only defining the easiest one for starters. Costmap
  // generator and planner coordinate what to use

  Eigen::Matrix2d discrete_costmap = Eigen::Matrix2d::Zero();
};

class IPlanner {
  virtual Path planIteration(InitialState state, CostMap cost, Path path, Zones zones) = 0;
};