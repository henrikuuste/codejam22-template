#include "config.h"
#include <iostream>
#include <pathplanning/pathplanning.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

int test(int val) {
  int i = 0x7fffffff;
  i += val;
}

int main(int argc, const char **argv) {
  spdlog::stopwatch sw;
  std::cout << "======================\n";
  std::cout << "\033[0;33mTest application " << codejam22::app::project_version << "\033[0m\n";
  std::cout << "======================\n";
  spdlog::info("Planning library version {}", pathplanning::version());
  pathplanning::plan();
  spdlog::warn("Testing {}", test(2));
  std::cout << "======================\n";
  spdlog::warn("Done {}", sw);

  int stack_array[100];
  return stack_array[100 + argc];
}