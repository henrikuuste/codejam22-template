#include "config.h"
#include <iostream>
#include <pathplanning/pathplanning.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <array>
using namespace pathplanning;

struct SimpleCostProvider : ICostProvider {
  // task1
  // create and store environment costmap
  // clang-format off
  std::vector<std::vector<float>> env{
  {0, 0, 0, 0, 0, 0}, 
  {0, 1, 1, 0, 0, 1}, 
  {0, 1, 1, 0, 1, 0},
  {0, 0, 0, 0, 0, 0}, 
  {1, 1, 1, 0, 1, 0}, 
  {1, 1, 1, 0, 0, 0}};
  // clang-format on

  CostOrError costBetween(StateQuery const &query) override {
    CostOrError cost = costOfTerrain(query);
    return cost;
  }
  CostOrError costOfStateChange(StateQuery const &query) override { return INTERNAL_ERROR; }
  CostOrError costOfEnvTraversal(StateQuery const &query) override { return INTERNAL_ERROR; }
  CostOrError costOfTerrain(StateQuery const &query) override {
    auto loc_from = query.from.loc();
    auto env_from = env.at(static_cast<size_t>(loc_from.y())).at(static_cast<size_t>(loc_from.x()));
    auto loc_to   = query.to.loc();
    auto env_to   = env.at(static_cast<size_t>(loc_to.y())).at(static_cast<size_t>(loc_to.x()));
    return Cost(env_to - env_from); // random value
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
  bool const operator>(const AStarPath &p) { return (f_score > p.f_score); }
  bool const operator<(const AStarPath &p) { return (f_score < p.f_score); }

  // std::vector<AStarPath> getNeighbours(std::weak_ptr<ICostProvider> provider) {
  //   std::vector<AStarPath> neighbours;

  //   return neighbours;
  // }
};

struct SimplePlanner : IPlanner {
  SimplePlanner() = default;
  PathOrError plan(State const &initial, TargetList const &targets) override {
    spdlog::stopwatch sw;
    // auto provider      = static_cast<SimpleCostProvider>(*cost_provider.lock());
    auto const &target = targets.at(0);

    AStarPath path;
    Waypoint start = {initial};
    path.path.emplace_back(start);
    path.f_score = target->heuristic(path);
    path.g_score = Cost(0);
    open_set.emplace_back(path);
    std::make_heap(open_set.begin(), open_set.end());
    while (!open_set.empty()) {

      // Choose path with lowest f score
      auto current = open_set.back();
      open_set.pop_back();
      // Check time limit
      if (sw.elapsed().count() > time_limit) {
        return current;
      }
      // Check if goal
      if (target->fitness(current)) {
        return current;
      }

      // auto neighbours = provider.ge
      // Expand
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
  IPlanner *setTimeLimit(seconds_t limit) override { return this; }

private:
  std::weak_ptr<ICostProvider> cost_provider;

  std::vector<AStarPath> open_set;
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

  SimpleCostProvider::CostOrError cost = cost_provider->costBetween({initial_state, goal_state});
  if (cost.has_value()) {
    std::cout << cost.value() << "\n";
  } else {
    std::cout << "Sum Ding Wong"
              << "\n";
  }

  // task 2 - integrate this
  // create planner object
  // make planner plan from initial state to goal state
  // planner.plan(costProvider, initialState, targetState)
  SimplePlanner planner;

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

  std::vector<std::vector<bool>> path_map;
  for (auto const &i : map) {
    path_map.push_back(std::vector<bool>());
    for (auto const &j : i) {
      path_map.back().emplace_back(false);
    }
  }

  if (path.has_value()) {

    for (auto const &wp : path.value().path) {
      auto loc                                                                   = wp.target.loc();
      path_map.at(static_cast<size_t>(loc.y())).at(static_cast<size_t>(loc.x())) = true;
    }

    for (size_t i = 0; i < map.size(); i++) {
      for (size_t j = 0; j < map.at(i).size(); j++) {
        if (path_map.at(i).at(j)) {
          std::cout << "\033[1;31m" << map.at(i).at(j) << "\033[0m";
        } else {
          std::cout << map.at(i).at(j);
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
