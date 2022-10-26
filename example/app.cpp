#include "config.h"
#include <iostream>
#include <pathplanning/pathplanning.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <array>
#include <queue>
using namespace pathplanning;

struct SimpleCostProvider : ICostProvider {
  // task1
  // create and store environment costmap
  // clang-format off
  Eigen::MatrixXd env{
  {0, 0, 0, 0, 0, 0}, 
  {0, 1, 0, 1, 0, 1}, 
  {0, 1, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 0}, 
  {1, 0, 1, 0, 1, 0}, 
  {1, 1, 0, 0, 0, 0}};
  // clang-format on

  CostOrError costBetween(StateQuery const &query) override {
    // std::cout << query << "\n";
    if (isOutOfBounds(query)) {
      // std::cout << "Out of bounds\n";
      return Cost(std::numeric_limits<double>::max(), Cost::UNKNOWN);
    }

    CostOrError cost = costOfEnvTraversal(query);
    return cost;
  }
  CostOrError costOfStateChange([[maybe_unused]] StateQuery const &query) override {
    return unexpected<Error>{Error::INTERNAL_ERROR};
  }
  CostOrError costOfEnvTraversal([[maybe_unused]] StateQuery const &query) override {
    auto loc_from = query.from.loc();
    auto env_from = env(static_cast<long>(loc_from.y()), (static_cast<long>(loc_from.x())));
    auto loc_to   = query.to.loc();
    auto env_to   = env(static_cast<long>(loc_to.y()), static_cast<long>(loc_to.x()));
    return Cost(env_to - env_from);
  }
  CostOrError costOfTerrain(StateQuery const &query) override {
    return unexpected<Error>{Error::INTERNAL_ERROR};
    }
  [[nodiscard]] StateBounds bounds() const override {
    State min_state;
    min_state.setStateElement(0, 0);
    min_state.setStateElement(0, 1);
    State max_state;
    max_state.setStateElement(5, 0);
    max_state.setStateElement(5, 1);
    return {min_state, max_state};
  }

  bool isOutOfBounds(StateQuery const &query) const {
    auto state_bounds = bounds();
    return (query.to > state_bounds.max() || query.to < state_bounds.min());
  }

  [[nodiscard]] SearchSpace const &searchSpace() const override { return {}; }

  // std::vector<State> get_neighbours(State current) {
  //   std::vector<State> neighbours;
  //   std::vector<Vec2> movement{Vec2(1, 0), Vec2(0, 1), Vec2(-1, 0), Vec2(0, -1)};

  //   for (auto const &i : movement) {
  //     State neighbour;
  //     neighbour.setLoc(current.loc() + i);
  //     if
  //       neighbours.emplace_back(neighbour);
  //   }

  //   return neighbours;
  // }
};

struct AStarPath : Path {
  Cost f_score = Cost(std::numeric_limits<double>::max());
  Cost g_score = Cost(std::numeric_limits<double>::max());
  bool operator>(const AStarPath &p) const { return (f_score > p.f_score); }
  bool operator<(const AStarPath &p) const { return (f_score < p.f_score); }

  // std::vector<AStarPath> getNeighbours(std::weak_ptr<ICostProvider> provider) {
  //   std::vector<AStarPath> neighbours;

  //   return neighbours;
  // }
};

class AStarPathComparator {
public:
  int operator()(const AStarPath &p1, const AStarPath &p2) const { return p1.f_score > p2.f_score; }
};

struct SimplePlanner : IPlanner {
  SimplePlanner() = default;
  PathOrError plan(State const &initial, TargetList const &targets) override {
    spdlog::stopwatch sw;
    auto provider      = std::static_pointer_cast<SimpleCostProvider>(cost_provider.lock());
    auto const &target = targets.at(0);

    AStarPath path;
    Waypoint start = {initial};
    path.emplace_back(start);
    path.f_score = target->heuristic(path);
    path.g_score = Cost(0);
    open_set.push(path);
    while (!open_set.empty()) {
      // Choose path with lowest f score
      auto current = open_set.top();
      open_set.pop();
      // Check time limit
      if (sw.elapsed().count() > time_limit) {
        return current;
      }
      // Check if goal
      if (target->satisfiesCriteria(current)) {
        return current;
      }

      // TODO break this into pieces
      const std::array<Vec2, 4> movement{Vec2(1, 0), Vec2(0, 1), Vec2(-1, 0), Vec2(0, -1)};
      State current_state = current.back().target;
      for (auto const &i : movement) {
        State neighbour;
        neighbour.setLoc(current_state.loc() + i);
        auto cost = provider->costBetween(StateQuery(current_state, neighbour));
        if (cost.has_value()) {
          if (cost.value().type() != Cost::UNKNOWN) {
            AStarPath new_path = current;
            Waypoint wp;
            wp.target = neighbour;
            new_path.emplace_back(wp);

            new_path.g_score = new_path.g_score + cost.value();
            new_path.f_score = new_path.g_score;
            new_path.f_score = new_path.f_score + target->heuristic(new_path);

            open_set.push(new_path);
          }
        }
      }
    }
    // for (auto const &target : targets) {
    //   // std::cout << target->heuristic(path);
    // }

    return unexpected<Error>{Error::INTERNAL_ERROR};
  }
  IPlanner *setCostProvider(std::weak_ptr<ICostProvider> provider) override {
    cost_provider = provider;
    return this;
  }
  IPlanner *setTimeLimit(seconds_t limit) override {
    time_limit = limit;
    return this;
  }

private:
  std::priority_queue<AStarPath, std::vector<AStarPath>, AStarPathComparator> open_set;
  seconds_t time_limit = 1;
};

int main() {
  spdlog::stopwatch sw;
  std::cout << "======================\n";
  std::cout << "\033[0;33mTest application " << codejam22::app::project_version << "\033[0m\n";
  std::cout << "======================\n";
  spdlog::info("Planning library version {}", pathplanning::version());
  std::cout << "======================\n";

  // task 1.5
  // create cost provider using costmap
  // define initial vehicle state
  // define goal state
  State initial_state;
  initial_state.setStateElement(0, 0);
  initial_state.setStateElement(0, 1);
  // std::cout << initial_state.getState() << "\n";
  State goal_state;
  goal_state.setStateElement(5, 0);
  goal_state.setStateElement(5, 1);
  // std::cout << goal_state.getState() << "\n";

  auto cost_provider = std::make_shared<SimpleCostProvider>();

  // SimpleCostProvider::CostOrError cost = cost_provider->costBetween({initial_state, goal_state});
  // if (cost.has_value()) {
  //   std::cout << cost.value() << "\n";
  // } else {
  //   std::cout << "Sum Ding Wong"
  //             << "\n";
  // }

  // task 2 - integrate this
  // create planner object
  // make planner plan from initial state to goal state
  // planner.plan(costProvider, initialState, targetState)
  SimplePlanner planner;
  planner.setTimeLimit(5);
  planner.setCostProvider(std::weak_ptr<SimpleCostProvider>(cost_provider));
  TargetList targets;
  Vec2 target_loc(5, 5);
  target_loc << 5, 5;
  PointTarget point_target(target_loc);
  State::Distance allowed_error;
  allowed_error.loc_distance = Vec2(0.5, 0.5);
  point_target.setAllowedError(allowed_error);
  targets.emplace_back(std::make_unique<PointTarget>(point_target));
  SimplePlanner::PathOrError path = planner.plan(initial_state, targets);
  // task 3
  // output path and costmap to something
  // Make a boolean map marking the points which path visits
  auto map = cost_provider->env;

  Eigen::MatrixXi path_map(2, 2);
  path_map.resize(map.rows(), map.cols());

  for (auto i = 0; i < map.rows(); i++) {
    for (auto j = 0; j < map.cols(); j++) {
      path_map(i, j) = 0;
    }
  }

  if (path.has_value()) {

    for (auto const &wp : path.value().path) {
      auto loc = wp.target.loc();
      path_map(static_cast<Eigen::Index>(loc.y()), static_cast<Eigen::Index>(loc.x())) = 1;
    }

    for (auto i = 0; i < map.rows(); i++) {
      for (auto j = 0; j < map.cols(); j++) {
        if (path_map(i, j)) {
          std::cout << "\033[1;31m" << map(i, j) << "\033[0m";
        } else {
          std::cout << map(i, j);
        }
      }
      std::cout << "\n";
    }
  } else {
    std::cout << "Sum Ding Wong"
              << "\n";
  }

  // task 4
  // when done, open up path in jupyter notebook
  // visualise results to validate behaviour

  spdlog::warn("Done {}", sw);
}
