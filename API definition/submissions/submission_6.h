enum Planner {
  astar_with_point_particle_and_inflation,
  astar_with_vehicle_dimensions,
  whatever
  // adding here should be backwards compatible
};

struct GoalPoint {
  Position position;
  float acceptable_distance;
};

struct State {
  std::shared_ptr<Cell> cell;
  Orientation orientation;
  Configuration configuration;
};

struct Vehicle {
  Size dimensions; // assume constant symmetrical box shaped vehicle
  float minimum_turn_radius;
}

struct Settings {
  Planner planner;
  Vehicle vehicle;

  // planning limitations
  float max_planning_time_seconds;

  // planning fitness criteria
  float path_shortness_weight;
  float path_smoothness_weight;
  std::function<float(State &, State &)> distance;
};

/*
 * Minumum 2 waypoints in: origin -> goal.
 * deque for popping the first and then replanning with updated map.
 * could add some concepts to limit MapT
 * I prefer exceptions to result enum, so this will throw whenever fails to plan
 * Don't care about the vehicle's speed capabilities, just create good path.
 */
template <typename MapT>
void plan(const std::deque<GoalPoint> &waypoints_in, std::vector<State> &waypoints_out,
          const Orientation &initial_orientation, const Settings &settings, const MapT &map) const;

// Example implementation
using Position    = Eigen::Vector2f;
using Size        = Eigen::Vector3f;
using Orientation = float;

struct Cell {
  Position position;
  std::vector<std::shared_ptr<Cell>> edges;
};

struct Map {
  Position center;
  Position size;
  Position cell_size;
  // where you can move from here and with what cost
  std::vector<std::pair<std::shared_ptr<Cell>, float>> cells;
};

// Example distance definition
float angle_sensitivity         = 0.1f;
float configuration_sensitivity = 999.0f;

static inline float angle_diff(float a, float b) {
  return abs(fmod(fmod(a - b + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI) - M_PI);
}

static inline float configuration_diff(Configuration a, Configuration b) { return 0.0f; }

static float distance(State &a, State &b) {
  std::array<float, 3> parts{(a.cell->position - b.cell->position).norm(),
                             angle_sensitivity * angle_diff(a.orientation, b.orientation),
                             configuration_sensitivity *
                                 configuration_diff(a.configuration, b.configuration)};

  return *std::max_element(parts.begin(), parts.end());
}
