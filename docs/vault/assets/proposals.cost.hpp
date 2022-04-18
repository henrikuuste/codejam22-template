/*******************
 * @Proposal 1a
 *******************/
struct Node {
  float cost;
};

/*******************
 * @Proposal 2a
 *******************/
float heuristic(/*...*/);
MatrixXd map;
using HeuristicCost = float;
using MapCost       = double;

/*******************
 * @Proposal 3
 *******************/
using Cost = double;

/*******************
 * @Proposal 4
 *******************/
using Cost = double;

/*******************
 * @Proposal 5
 *******************/
using Cost                         = uint8_t;
static constexpr Cost freeCost     = 0;
static constexpr Cost occupiedCost = 250;
/* 251-255 reserved/unknown */

/*******************
 * @Proposal 6
 *******************/
// No cost definition provided
// Final submission
struct Cell {
  Position position;
  std::vector<std::shared_ptr<Cell>> edges;
};

// Initial submission example uses costmap_2d::Costmap2D
using Cost = unsigned char;

/*******************
 * @Proposal 7
 *******************/
using Cost = double;

/*******************
 * @Proposal 8
 *******************/
using Cost = double;