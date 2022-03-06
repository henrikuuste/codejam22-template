#include <iostream>
#include <numbers>
#include <pathplanning/pathplanning.h>
#include <spdlog/spdlog.h>

namespace pathplanning {
int plan() {
  std::cout << "\033[0;31mPlanning your demise\033[0m\n";
  spdlog::info("Have some pi in the meanwhile {}", std::numbers::pi);
  return 128;
}
} // namespace pathplanning