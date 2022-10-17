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
  std::vector<std::vector<float>> env{{0, 0, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 1}, {0, 1, 1, 0, 1, 0},
                                      {0, 0, 0, 0, 0, 0}, {1, 1, 1, 0, 1, 0}, {1, 1, 1, 0, 0, 0}};

  CostOrError costBetween(StateQuery const &query) override {
    CostOrError cost = costOfTerrain(query);
    return cost;
  }
  CostOrError costOfStateChange(StateQuery const &query) override { return INTERNAL_ERROR; }
  CostOrError costOfEnvTraversal(StateQuery const &query) override { return INTERNAL_ERROR; }
  CostOrError costOfTerrain(StateQuery const &query) override {
    auto from_state = query.from.getState();
    auto env_from   = env.at(from_state(1)).at(from_state(0));
    auto to_state   = query.to.getState();
    auto env_to     = env.at(to_state(1)).at(to_state(0));

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
};

struct SimplePlanner : IPlanner {
  PathOrError plan(State const &initial, TargetList const &targets) override {
    return unexpected<Error>{Error::INTERNAL_ERROR};
  }
  IPlanner *setCostProvider(ICostProvider *provider) override {
    cost_provider = provider;
    return this;
  }
  IPlanner *setTimeLimit(seconds_t limit) override { return this; }

private:
  ICostProvider *cost_provider;
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

  SimpleCostProvider cost_provider;
  SimpleCostProvider::CostOrError cost = cost_provider.costBetween({initial_state, goal_state});
  if (cost.has_value()) {
    std::cout << cost.value() << "\n";
  } else {
    std::cout << "Sumding Wong"
              << "\n";
  }
  SimplePlanner planner;
  planner.setCostProvider(&cost_provider);
  TargetList targets;
  Vec2 target_loc;
  target_loc << 5, 5;
  PointTarget point_target(target_loc);
  targets.emplace_back(std::make_unique<PointTarget>(point_target));
  SimplePlanner::PathOrError result = planner.plan(initial_state, targets);
  // task 2 - integrate this
  // create planner object
  // make planner plan from initial state to goal state
  // planner.plan(costProvider, initialState, targetState)

  // task 3
  // output path and costmap to something

  // task 4
  // when done, open up path in jupyter notebook
  // visualise results to validate behaviour

  spdlog::warn("Done {}", sw);
}
