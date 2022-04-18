
/*
input costmap should be nd
https://en.wikipedia.org/wiki/A*_search_algorithm#/media/File:Weighted_A_star_with_eps_5.gif
*/
#include <vector>

namespace pathplanner {
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
// 'using' stuff is as an example!
// all of using stuff is an example of user-defined stuff

// Basic types should be defined in this library
// If user wants to use some other State component (e.g. payload), where should that be
// implemented?
using StateT = {Position<dim>, Orientation<dim>, Velocity<dim>, Payload};

// Position is either 2 or 3 element struct, depending on dimensions
using PositionT = Position<dim>;

// PathElementT is the minimum needed info for following the path
// PathElementT is somewhere between PositionT and StateT - PositionT <= PathElementT <= StateT
using PathElementT = {PositionT, Payload};

// CostMap is a n-dimensional grid of
using CostMap = ; // Costmap<Node, dim>

// Pathplanner main class
struct PathPlanner {
  using Path = std::vector<PathElementT>;
  // CostMap is
  const CostMap staticCostmap;
  // SomeGraph dynamicCostmap;

  // predictState should use vehicle physical model to predict the state at the next node depending
  // on current state and next nodes position physical model should be in a separate library (same
  // physical model for UKF and path planner etc in a separate library)
  // void setInitialState(StateT state);
  // void setGoal(PositionT postition);
  StateT predictState(StateT currentState, PositionT nextPosition);

  // queryDynamicCost will invoke predictState and queryStaticCost
  // will calculate the cost based on physical model and static costs
  DynamicCost queryDynamicCost(StateT currentState, PositionT position);

  // retrieve input costmap node based on whatever input (position or index or whatever)
  Node queryStaticCost();

  // planPath will invoke queryDynamicCost to build the necessary graph for path planning
  // and performs the selected algorithm (A* or sth) at the same time
  Path planPath(StateT currentState, StateT desiredState);
};
} // namespace pathplanner