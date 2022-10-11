#include "config.h"
#include <iostream>
#include <pathplanning/pathplanning.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <array>
using namespace pathplanning;

struct SimpleCostProvider : ICostProvider {
  std::vector<std::vector<float>> env{{0, 0, 0, 0, 0, 0}, {0, 1, 1, 0, 0, 1}, {0, 1, 1, 0, 1, 0},
                                      {0, 0, 0, 0, 0, 0}, {1, 1, 1, 0, 1, 0}, {1, 1, 1, 0, 0, 0}};

  CostOrError costBetween(State const &from, State const &to) override {
    CostOrError cost = costOfTerrain(from, to);
    return cost;
  }
  CostOrError costOfStateChange(State const &from, State const &to) override {
    return INTERNAL_ERROR;
  }
  CostOrError costOfEnvTraversal(State const &from, State const &to) override {
    return INTERNAL_ERROR;
  }
  CostOrError costOfTerrain(State const &from, State const &to) override {
    auto from_state = from.getState();
    auto env_from   = env.at(from_state(1)).at(from_state(0));
    auto to_state   = to.getState();
    auto env_to     = env.at(to_state(1)).at(to_state(0));

    return Cost(env_to - env_from); // random value
  }
  StateBounds bounds() const override {
    State min_state;
    min_state.setStateElement(0, 0);
    min_state.setStateElement(0, 1);
    State max_state;
    max_state.setStateElement(5, 0);
    max_state.setStateElement(5, 1);
    return {min_state, max_state};
  }
  SearchSpace const &searchSpace() const override { return {}; }
};

int main() {
  spdlog::stopwatch sw;
  std::cout << "======================\n";
  std::cout << "\033[0;33mTest application " << codejam22::app::project_version << "\033[0m\n";
  std::cout << "======================\n";
  spdlog::info("Planning library version {}", pathplanning::version());
  std::cout << "======================\n";
  // task1
  // create and store environment costmap
  State initial_state;
  initial_state.setStateElement(0, 0);
  initial_state.setStateElement(0, 1);
  std::cout << initial_state.getState() << "\n";
  State goal_state;
  goal_state.setStateElement(5, 0);
  goal_state.setStateElement(5, 1);
  std::cout << goal_state.getState() << "\n";

  SimpleCostProvider cost_provider;
  SimpleCostProvider::CostOrError cost = cost_provider.costBetween(initial_state, goal_state);
  if (cost.has_value()) {
    std::cout << cost.value() << "\n";
  } else {
    std::cout << "Sumding Wong"
              << "\n";
  }
  // task 1.5
  // create cost provider using costmap
  // define initial vehicle state
  // define goal state

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