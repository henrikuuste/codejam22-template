/*******************
 * @Proposal 1a
 *******************/
// Static costmap cell should be user-defined (might be just single number cost of a single pixel or
// might be multiple factors such as slope or roughness or whatever )
/*
struct Node {
  float cost;
};
OR
struct Node {
  float roughness;
  float slope;
  bool landmine;
}
*/
struct Node {};
template <size_t Dim> struct CostMap {
  // would like to make this CostMap so that it can be initialized as n-D array with type Node
};
// CostMap is a n-dimensional grid of
using CostMap = ; // Costmap<Node, dim>

// Pathplanner main class
struct PathPlanner {
  // queryDynamicCost will invoke predictState and queryStaticCost
  // will calculate the cost based on physical model and static costs
  DynamicCost queryDynamicCost(StateT currentState, PositionT position);

  // retrieve input costmap node based on whatever input (position or index or whatever)
  Node queryStaticCost();
};

/*******************
 * @Proposal 2a
 *******************/
using heuristic = float (*)(MapCoordinate, MapCoordinate);
/**
 * A simple map class which holds a map in matrix format and its dimensions.
 * At the moment the map consists of integer coordinates only.
 */
class simpleMap {
  simpleMap(std::size_t rows, std::size_t cols)
      : map(std::make_shared<Eigen::MatrixXd>(rows, cols)), Y(map->rows()), X(map->cols()),
        minCost_(map->minCoeff()), maxCost_(map->maxCoeff()) {}

public:
  const int X; // x dimension of the map
  const int Y; // y dimension of the map
  const std::shared_ptr<Eigen::MatrixXd> map;
  const int minCost_; // Min cost value in the map
  const int maxCost_; // Max cost value in the map
};

/*******************
 * @Proposal 3
 *******************/
class Costmap {
public:
  using costmap_ptr_t = std::shared_ptr<std::vector<std::vector<double>>>;
  using costmap_t     = std::vector<std::vector<double>>;
  using Vector2i      = Eigen::Vector2i;

  static costmap_ptr_t createEmptyCostmap(const double width, const double height,
                                          const double resolution);

  Costmap(costmap_ptr_t &costmap, double const resolution, Vector2 origin);
  costmap_ptr_t &getCostmap();
  Vector2i positionToIndices(const Vector2 &position);
  void setCostAtPosition(const Vector2 position, const double value);
  double costAtPosition(const Vector2 &position) const;
  double costAtIndeces(const Vector2i &indices_xy);
};

/*******************
 * @Proposal 4
 *******************/
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

/*******************
 * @Proposal 5
 *******************/
/*
  Map
    including information who are neighbors
    what is distance cost of moving from one neighbor to another
    what is occupancy cost
      free area cost
      occupied area cost
      typical cost area being between free and occupied cost
      optionally unknown area cost should be defined
    Unknown area could be used to solve exploration problem differently
    from normal search or just set it to half the cost for example..
*/

/*
  potentially grid map where neighbors are cells next to each other
  occupancy cost is cell value
  distance cost would be euclidean distance between cells
*/

/*
  potentially graph where edges represent connections between neighbors
    (vertexes being a location where to move)
  edge including distance cost between neighbors
  vertexes including occupancy cost

  Another solution would be to include occupancy cost from one vertex
  to another in the edge cost and calculate distance cost based on vertex coordinates,
  which should be set in that case..
*/

/*
  uint8 matrix which represents grid map
  ...
  Distance cost amplifier - double value
*/
class CostMap {
  using Grid = Eigen::Matrix<Cost, Dynamic, Dynamic>;

  std::vector<GridIndex> getNeighbors(GridIndex const &idx) const;

  Cost distanceCost(GridIndex const &from, GridIndex const &to) const;
  Cost occupancyCost(GridIndex const &idx) const;

  bool isFree(GridIndex const &idx) const;
  bool isOccupied(GridIndex const &idx) const;
  bool isUnknown(GridIndex const &idx) const;

  Grid data_;
  double distance_cost_amplifier_ = 1.0;
};

/*******************
 * @Proposal 6
 *******************/
template <typename MapT> void plan(/*...*/);
// Final submission
struct Map {
  Position center;
  Position size;
  Position cell_size;
  // where you can move from here and with what cost
  std::vector<std::pair<std::shared_ptr<Cell>, float>> cells;
};

// Initial submission uses costmap_2d as example implementation
using Map = costmap_2d::Costmap2D;

namespace costmap2d {
class Costmap2D {
  unsigned char *getCharMap() const;
  unsigned char getCost(unsigned int mx, unsigned int my) const;
  unsigned char getDefaultValue();

  unsigned int getSizeInCellsX() const;
  unsigned int getSizeInCellsY() const;
  double getSizeInMetersX() const;
  double getSizeInMetersY() const;
  double getOriginX() const;
  double getOriginY() const;
  double getResolution() const;

  mutex_t *getMutex();
};
} // namespace costmap2d

/*******************
 * @Proposal 7
 *******************/
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
  virtual double getCostFromTo(Eigen::Vector2d source, Eigen::Vector2d target) const;
  virtual void setVehicleDimensions(double x, double y, double z) = 0;
};

/*******************
 * @Proposal 8
 *******************/
struct CostMap {
  // Will have multiple formats here, but only defining the easiest one for starters. Costmap
  // generator and planner coordinate what to use

  Eigen::Matrix2d discrete_costmap = Eigen::Matrix2d::Zero();
};