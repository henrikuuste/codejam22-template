#include <numbers>
#include <pathplanning/pathplanning.h>
#include <spdlog/spdlog.h>

#include "config.h"

namespace pathplanning {

std::string_view version() { return codejam22::lib::project_version; }

} // namespace pathplanning