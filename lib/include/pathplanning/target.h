#pragma once

#include "cost.h"
#include "internal/common.h"
#include "path.h"
#include "state.h"

#include <vector>

namespace pathplanning {

struct ITargetCriteria {
  using Fitness              = float; // TODO should this be == Cost?
  virtual ~ITargetCriteria() = default;

  /**
   * @brief Evaluate how well the given path fits/matches the target criteria
   *
   * @return Fitness values <= 0 mean that the path does not satisfy the criteria
   */
  virtual Fitness fitness(Path const &path) const = 0;

  /**
   * @brief By default, a given path satisfies the target criteria if the fitness is above 0
   *
   * @return true The given path satisfies the target criteria
   */
  virtual bool satisfiesCriteria(Path const &path) const { return fitness(path) > 0.f; };

  /**
   * @brief The allowed error is set based on the internal structure of the planner to account for
   * things like grid resolution
   */
  virtual ITargetCriteria &setAllowedError(State::Distance const &error) = 0;

  /**
   * @brief Provide a heuristic value to guide the search towards the target
   *
   * @param path The current path from the search algorithm
   * @return Cost Heuristic value as a cost that can be compared
   */
  virtual Cost heuristic(Path const &path) const = 0;
};

using TargetList = std::vector<std::unique_ptr<ITargetCriteria>>;

} // namespace pathplanning