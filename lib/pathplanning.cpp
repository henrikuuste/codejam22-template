#include <numbers>
#include <pathplanning/pathplanning.h>
#include <spdlog/spdlog.h>

#include "config.h"

namespace pathplanning {
int plan() {
  spdlog::error("Planning your demise");
  spdlog::info("Have some pi in the meanwhile {}", std::numbers::pi);
  return 128;
}

std::string_view version() { return codejam22::lib::project_version; }

} // namespace pathplanning