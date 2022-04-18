#pragma once

#include <eigen3/Eigen/src/Core/Matrix.h>
#include <memory>
#include <vector>

class Costmap {
public:
  using costmap_ptr = std::shared_ptr<std::vector<std::vector<double>>>;
  using costmap     = std::vector<std::vector<double>>;
  using vector2d    = Eigen::Vector2d;
  using vector2i    = Eigen::Vector2i;

  costmap_ptr getCostmap();

  double getWidth();
  double getHeight();
  double getResolution();
  vector2d getOrigin();

  vector2d getPosition(const vector2i &index);
  vector2i getIndex(const vector2d &position);

  bool isObstacle(const vector2i &index);
  bool isObstacle(const vector2d &position);
  bool isInflation(const vector2i &index);
  bool isInflation(const vector2d &position);
  bool isPassable(const vector2i &index);
  bool isPassable(const vector2d &position);
  double costAtIndex(const vector2i &index);
  double costAtPosition(const vector2d &position);
};

class Pathplanner {
public:
  using position = Eigen::Vector2d;
  using path     = std::vector<position>;

  struct result {
    double total_time;
    double total_length;
    path path_list;
  };

  struct pathConfig {
    bool shortest_time;
    bool shortest_distance;
    double target_speed;
    double max_speed;
    int nr_of_alternate_routes;
  };

  struct machineConfig {
    Eigen::Vector2d machine_dimensions;
  };

  struct waypoint {
    Eigen::Vector2d position;
    double acceptable_radius;
  };

  std::vector<result> createPlan(const Costmap &costmap, const waypoint &current_pos,
                                 const waypoint &goal_pos, const pathConfig &path_cfg,
                                 const machineConfig &vehicle_cfg);
};