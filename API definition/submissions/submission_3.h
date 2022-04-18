#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

// Free to use standard vector algebra
using Position = Eigen::Vector2d;

class Costmap {
public:
  using costmap_ptr_t = std::shared_ptr<std::vector<std::vector<double>>>;
  // Cost is double - not sure what it means tho
  using costmap_t = std::vector<std::vector<double>>;
  using Vector2i  = Eigen::Vector2i;

  static costmap_ptr_t createEmptyCostmap(const double width, const double height,
                                          const double resolution);

  Costmap(costmap_ptr_t &costmap, double const resolution, Vector2 origin);
  costmap_ptr_t &getCostmap();
  // Reference to a shared_ptr?
  // makePlan can not access this
  Vector2i positionToIndices(const Vector2 &position);
  void setCostAtPosition(const Vector2 position, const double value);
  // Set single element cost
  // Not used by planner, since it can not modify the cost map
  double costAtPosition(const Vector2 &position) const;
  // Currently makePlan does not have access to this function
  double costAtIndeces(const Vector2i &indices_xy);
  // Assuming that cost map is discretized
  // bool isObstacle(Vector2 position) const;
};

class BasePathPlanner {
public:
  using Vector2 = Eigen::Vector2d;
  using Path    = std::vector<Vector2>;

  virtual Path makePlan(Vector2 const &current_position, Vector2 const &goal_position,
                        Costmap const &costmap) = 0;
  // Assuming static cost
  // Single goal/waypoint planner
  // No assumptions or control over vehicle dimensions
  // No assumptions on heurstic
  // Planner is stateful
  // No way of reporting errors
};