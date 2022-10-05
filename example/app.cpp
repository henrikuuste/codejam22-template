#include "config.h"
#include <iostream>
#include <pathplanning/pathplanning.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

int main() {
  spdlog::stopwatch sw;
  std::cout << "======================\n";
  std::cout << "\033[0;33mTest application " << codejam22::app::project_version << "\033[0m\n";
  std::cout << "======================\n";
  spdlog::info("Planning library version {}", pathplanning::version());
  std::cout << "======================\n";
  // task1
  // create and store environment costmap

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