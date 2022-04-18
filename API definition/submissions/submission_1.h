typedef SthSthMagicGraph Graph;
// Incomplete type
// typedef - prefer using
typedef Path std::vector<size_t>;
using Path std::vector<Waypoint>;
// List of indices
// Index into something? Graph? Need an interface for that
// Domain point of view - a single index is bad
enum PlanningMode { SEQUENTIAL, SHORTEST_PATH };
// Assuming and limiting implementation
enum PlannerType { ASTAR, RRT };
struct PPConfig {
  PlannerType plannerType   = ASTAR;
  PlanningMode planningMode = SEQUENTIAL;
};
struct Waypoint {
  size_t idx;
};
Path plan(Graph &cmap, size_t start_idx, Path waypoints, PPConfig config);
// Planner has no state - stateless
// Plan multiple waypoints
//   Assuming sequential or TSP based on config
// No assumption on cost
// Assuming a discretized map - indices
// No assumption on the vehicle
// Loose assumption on the heuristic (shortest distance?)
//   How do we measure distance?
