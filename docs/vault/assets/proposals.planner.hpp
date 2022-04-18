/*******************
 * @Proposal 1a
 *******************/
// Pathplanner main class
struct PathPlanner {
  using Path = std::vector<PathElementT>;
  Path planPath(StateT currentState, StateT desiredState);
};

/*******************
 * @Proposal 2a
 *******************/
/**
 * Result of the planner if a valid path can be found or not.
 */
enum Result {
  SOLVED,           // A solution was found
  UNSOLVED,         // No solution was found
  INVLMAPFORMAT,    // Map is null
  LONGPATHSOLVED,   // A solution was found but the lenght of the path is longer than the threshold
  LONGPATHUNSOLVED, // No solution was found even beyond the path threshold was included in the
                    // search
  OUTOFMAP,         // Given start or end coordinates out of the map
  INVLCOOR          // Coordinates are null
};

using Path = std::vector<MapCoordinate>;

class basePlanner {
public:
  struct pathPlannerResult {
  public:
    MapCoordinate startPoint() { return startPoint_; }
    MapCoordinate endPoint() { return endPoint_; }
    Path paths() { return paths_; }
    Result result() { return result_; }

    MapCoordinate setStartPoint(MapCoordinate sp) { startPoint_ = sp; }
    MapCoordinate setEndPoint(MapCoordinate ep) { endPoint_ = ep; }
    Path setPaths(Path p) { paths_ = p; }
    Result setResult(Result r) { result_ = r; }

  private:
    MapCoordinate startPoint_; // The starting point of the planning
    MapCoordinate endPoint_;   // The destination point of the planning
    Path paths_;               // Points of the solved path
    Result result_;            // Success result of the path planning, if there is any valid path
  };

  virtual pathPlannerResult solver(const simpleMap &map, MapCoordinate startPoint,
                                   MapCoordinate endPoint, int obstacleThreshold = 0,
                                   std::size_t maxPathLength = 0, bool cancelAtMaximum = false) = 0;
};

/*******************
 * @Proposal 3
 *******************/
class BasePathPlanner {
public:
  using Path = std::vector<Vector2>;

  virtual Path makePlan(Vector2 const &current_position, Vector2 const &goal_position,
                        Costmap const &costmap) = 0;
};

/*******************
 * @Proposal 4
 *******************/
class Pathplanner {
public:
  using path = std::vector<position>;

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

  std::vector<result> createPlan(const Costmap &costmap, const waypoint &current_pos,
                                 const waypoint &goal_pos, const pathConfig &path_cfg,
                                 const machineConfig &vehicle_cfg);
};

/*******************
 * @Proposal 5
 *******************/
/*
- Inputs
  - Map - uint8 matrix which represents grid map.
  - Starting location - matrix x and y coordinates.
  - Goal location - matrix x and y coordinates.
  - Distance cost amplifier - double value
  - Max search time - double value
  - Max path distance - double value
  - Max cost value - double value
- Outputs
  - Optimal path - array of matrix coordinates.
  - Return code - enum
    - 0 - successful search
    - 1 - path not achievable
    - 2 - max search time exceeded
    - 3 - max path distance exceeded
    - 4 - max cost value exceeded
    - 5 - starting location not on map
    - 6 - goal location not on map
*/

/*******************
 * @Proposal 6
 *******************/
enum Planner {
  astar_with_point_particle_and_inflation,
  astar_with_vehicle_dimensions,
  whatever
  // adding here should be backwards compatible
};

struct Settings {
  Planner planner;
  Vehicle vehicle;

  // planning limitations
  float max_planning_time_seconds;

  // planning fitness criteria
  float path_shortness_weight;
  float path_smoothness_weight;
};

/*
 * Minumum 2 waypoints in: origin -> goal.
 * deque for popping the first and then replanning with updated map.
 * could add some concepts to limit MapT
 * I prefer exceptions to result enum, so this will throw whenever fails to plan
 * Don't care about the vehicle's speed capabilities, just create good path.
 */
template <typename MapT>
void plan(const std::deque<GoalPoint> &waypoints_in, std::vector<WayPoint> &waypoints_out,
          const float initial_orientation, const Settings &settings, const MapT &map) const;

/*******************
 * @Proposal 7
 *******************/
class IPathPlanner {
public:
  IPathPlanner() = default;
  virtual int plan(std::shared_ptr<MapType> map, Pos start, Pos goal, std::vector<Pos> &path) = 0;
  virtual int plan(std::shared_ptr<MapType> map, std::vector<Pos> startToEndPositions,
                   std::vector<Pos> &path)                                                    = 0;
};

/*******************
 * @Proposal 8
 *******************/
struct Path {
  std::vector<Waypoint> waypoints;
};

class IPlanner {
  virtual Path planIteration(InitialState state, CostMap cost, Path path, Zones zones) = 0;
};