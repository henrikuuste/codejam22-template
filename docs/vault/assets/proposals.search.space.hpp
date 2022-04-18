/*******************
 * @Proposal 1a
 *******************/
// Basic types should be defined in this library
// If user wants to use some other State component (e.g. payload), where should that be
// implemented?
using StateT = {Position<dim>, Orientation<dim>, Velocity<dim>, Payload};

// Position is either 2 or 3 element struct, depending on dimensions
using PositionT = Position<dim>;

// PathElementT is the minimum needed info for following the path
// PathElementT is somewhere between PositionT and StateT - PositionT <= PathElementT <= StateT
using PathElementT = {PositionT, Payload};

using Path = std::vector<PathElementT>;

/*******************
 * @Proposal 2a
 *******************/
using MapCoordinate = Eigen::Vector2f;
using Path          = std::vector<MapCoordinate>;

/*******************
 * @Proposal 3
 *******************/
using Position = Eigen::Vector2d;
using Path     = std::vector<Vector2>;

/*******************
 * @Proposal 4
 *******************/
using position = Eigen::Vector2d;
using path     = std::vector<position>;

struct waypoint {
  Eigen::Vector2d position;
  double acceptable_radius;
};

/*******************
 * @Proposal 5
 *******************/
/*
  - Starting location - matrix x and y coordinates.
  - Goal location - matrix x and y coordinates.
  - Optimal path - array of matrix coordinates.
 */
using Location = Eigen::Matrix<double, 2, 1>;
using Location = Eigen::Matrix<size_t, 2, 1>;
using Path     = std::vector<Location>; // or Matrix<foo, 2, Dynamic>

/*******************
 * @Proposal 6
 *******************/
using Position = Eigen::Vector2f;

struct GoalPoint {
  Position position;
  float acceptable_radius;
};

struct WayPoint {
  Position position;
  float orientation;
};

using Goals = std::deque<GoalPoint>;
using Path  = std::vector<WayPoint>;

/*******************
 * @Proposal 7
 *******************/
/*
 * pos - represents robot position on a 2D plane.
 * level - plane identifier. Multiple planes can be useful for navigating
 * under/over a bridge and on multilevel buildings.
 */
struct Pos {
  Eigen::Vector2d pos;
  int level;
};

using GoalType = Pos;
using Path     = std::vector<Pos>;

/*******************
 * @Proposal 8
 *******************/
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

// NOTE Path is used for both goal input and output
struct Path {
  std::vector<Waypoint> waypoints;
};

struct InitialState {
  Vector3 pos       = Vector3::Zero();
  Quaternion orient = Quaternion::Identity();
};
