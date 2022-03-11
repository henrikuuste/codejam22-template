#pragma once

#include <pathplanning/export.h>
#include <string_view>

namespace pathplanning {

/**
 * @brief Plan evil things
 *
 * @return int your demise
 */
PATHPLANNING_EXPORT int plan();

PATHPLANNING_EXPORT std::string_view version();

} // namespace pathplanning