#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace planner {

/*
 * pos - represents robot position on a 2D plane.
 * level - plane identifier. Multiple planes can be useful for navigating
 * under/over a bridge and on multilevel buildings.
 */
struct Pos {
  Eigen::Vector2d pos;
  int level;
};

class ICostMap {
protected:
  Eigen::Vector3d vehicle_dim_;

public:
  ICostMap() = default;
  /*
   * Cost between two positions, measured in a straight line
   */
  virtual double getCostFromTo(Pos source, Pos target) const = 0;

  /*
   * Cost between two positions, measured in a straight line
   * Possible rotation allowed during movement for holonomic robots.
   */
  virtual double getCostFromTo(Pos source, Pos target, Eigen::Quaterniond sourceRot,
                               Eigen::Quaterniond targetRot) const = 0;

  virtual double getCostFromTo(Eigen::Vector2d source, Eigen::Vector2d target) const {
    Pos src;
    Pos tgt;
    src.pos = source;
    tgt.pos = target;
    return getCostFromTo(source, target);
  }

  virtual void setVehicleDimensions(double x, double y, double z) = 0;
};

using GoalType = Pos;
using MapType  = ICostMap;

class IPathPlanner {
public:
  IPathPlanner() = default;
  virtual int plan(std::shared_ptr<MapType> map, Pos start, Pos goal, std::vector<Pos> &path) = 0;
  virtual int plan(std::shared_ptr<MapType> map, std::vector<Pos> startToEndPositions,
                   std::vector<Pos> &path)                                                    = 0;
};

} // namespace planner