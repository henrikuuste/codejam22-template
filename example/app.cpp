#include "config.h"
#include <array>
#include <fstream>
#include <iostream>
#include <pathplanning/pathplanning.h>
#include <queue>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <sstream>
using namespace pathplanning;

void draw_path(const Eigen::MatrixXd &map, const Path &path) {
  // Create a binary matrix showing where the path is
  Eigen::MatrixXi path_map(2, 2);
  path_map.resize(map.rows(), map.cols());
  for (auto i = 0; i < map.rows(); i++) {
    for (auto j = 0; j < map.cols(); j++) {
      path_map(i, j) = 0;
    }
  }
  for (auto const &wp : path.path) {
    auto loc                                                                         = wp.loc();
    path_map(static_cast<Eigen::Index>(loc.y()), static_cast<Eigen::Index>(loc.x())) = 1;
  }
  for (long i = 0; i < map.rows(); i++) {
    for (long j = 0; j < map.cols(); j++) {
      if (path_map(i, j)) {
        // If point on path, mark it with red
        std::cout << "\033[1;31m" << map(i, j) << "\033[0m";
      } else {
        std::cout << map(i, j);
      }
    }
    std::cout << "\n";
  }
}

struct ImportedData {
  Eigen::MatrixXd cost_map;
  Vec2 start;
  Vec2 finish;
};

ImportedData import_from_csv() {
  std::cout << "Importing\n";
  ImportedData data;
  Eigen::MatrixXd map;
  std::ifstream f;
  std::string line, cell;
  std::vector<Eigen::Index> ends;
  Eigen::Index x = 0;
  Eigen::Index y = 0;

  f.open("input.csv");
  if (f.is_open()) {
    // Start and finish
    for (int i = 0; i < 4; i++) {
      if (i == 3) {
        getline(f, line);
      } else {
        getline(f, line, ',');
      }
      ends.emplace_back(std::stoi(line));
      line.clear();
    }
    // Mapsize
    getline(f, line, ',');
    x = std::stoi(line);
    line.clear();
    getline(f, line);
    y = std::stoi(line);
    line.clear();
    map.resize(y, x);
    x = 0;
    y = 0;
    // Cost
    while (getline(f, line)) {
      std::stringstream ss(line);
      while (getline(ss, cell, ',')) {
        map(y, x) = std::stod(cell);
        x++;
      }
      line.clear();
      x = 0;
      y++;
    }
  } else {
    std::cout << "Sum Ding Wong: no file"
              << "\n";
  }
  data.cost_map = map;
  data.start    = Vec2(ends[0], ends[1]);
  data.finish   = Vec2(ends[2], ends[3]);

  return data;
}

// First row of the output csv will contain the coordinates of the waypoints making up the path
// Next rows will contain the map
void export_to_csv(const Eigen::MatrixXd &map, const Path &path) {
  std::cout << "Exporting\n";
  std::ofstream f;
  f.open("output.csv");

  for (auto const &wp : path.path) {
    auto loc = wp.loc();
    f << loc.x() << "-" << loc.y() << ",";
  }
  f << "\n";
  for (long i = 0; i < map.rows(); i++) {
    for (long j = 0; j < map.cols(); j++) {
      f << map(i, j) << ",";
    }
    f << "\n";
  }
}

// For creating and storing costmap within the example planner
struct SimpleEnvironment {
  // clang-format off
  Eigen::MatrixXd env {
  {1, 1, 1, 1, 1, 1}, 
  {1, 1, 1, 1, 1, 1}, 
  {1, 1, 1, 2, 1, 1},
  {1, 1, 1, 1, 2, 2}, 
  {1, 1, 1, 1, 1, 1}, 
  {1, 1, 1, 3, 1, 1}};
  // clang-format on

  enum Error { OUT_OF_BOUNDS };
  using RealOrError = expected<Real, Error>;

  RealOrError getValue(Vec2 loc) {
    auto x_ = static_cast<Eigen::Index>(loc.x());
    auto y_ = static_cast<Eigen::Index>(loc.y());

    if (isOutOfBounds(x_, y_)) {
      return unexpected<Error>{Error::OUT_OF_BOUNDS};
    } else {
      return env(y_, x_);
    }
  };

  bool isOutOfBounds(Eigen::Index x, Eigen::Index y) {
    if (x < 0 || x > env.cols() - 1) {
      return true;
    }
    if (y < 0 || y > env.rows() - 1) {
      return true;
    }
    return false;
  };
};

struct SimpleCostProvider : ICostProvider {

  CostOrError costBetween(StateQuery const &query) override {
    CostOrError cost = costOfEnvTraversal(query);
    return cost;
  }
  CostOrError costOfStateChange([[maybe_unused]] StateQuery const &query) override {
    return unexpected<Error>{Error::INTERNAL_ERROR};
  }
  CostOrError costOfEnvTraversal([[maybe_unused]] StateQuery const &query) override {
    auto env_from = env.getValue(query.from.loc());
    auto env_to   = env.getValue(query.to.loc());
    if (env_from.has_value() && env_to.has_value()) {
      return Cost(abs(env_to.value() - env_from.value()));

    } else {

      return Cost(std::numeric_limits<double>::max(), Cost::UNKNOWN);
    }
  }
  CostOrError costOfTerrain(StateQuery const &query) override {
    return unexpected<Error>{Error::INTERNAL_ERROR};
  }
  [[nodiscard]] StateBounds bounds() const override { return {}; }

  [[nodiscard]] SearchSpace const &searchSpace() const override { return {}; }

  SimpleEnvironment env;
};

struct AStarPath : Path {
  Cost f_score = Cost(std::numeric_limits<double>::max());
  Cost g_score = Cost(std::numeric_limits<double>::max());
  bool operator>(const AStarPath &p) const { return (f_score > p.f_score); }
  bool operator<(const AStarPath &p) const { return (f_score < p.f_score); }

  bool point_on_path(State point) {
    for (State path_point : path) {
      if (point == path_point) {
        return true;
      }
    }
    return false;
  }
};

class AStarPathComparator {
public:
  bool operator()(const AStarPath &p1, const AStarPath &p2) const {
    return p1.f_score > p2.f_score;
  }
};

struct SimplePlanner : IPlanner {
  SimplePlanner() = default;

  PathOrError plan(State const &initial, TargetList const &targets) override {
    std::cout << "Running planner\n";
    spdlog::stopwatch sw;
    auto provider = std::static_pointer_cast<SimpleCostProvider>(cost_provider.lock());

    // Currently uses a single target
    auto const &target = targets.at(0);
    // TODO is target within bounds?

    // initialize current_g_costs
    std::vector<std::vector<Cost>> current_g_costs;
    for (size_t i = 0; i < provider->env.env.rows(); i++) {
      std::vector<Cost> row;
      for (size_t j = 0; j < provider->env.env.cols(); j++) {
        row.emplace_back(Cost(std::numeric_limits<double>::max(), Cost::UNKNOWN));
      }
      current_g_costs.emplace_back(row);
    }

    // Initial path setup
    if (provider->env.isOutOfBounds(initial.loc().x(), initial.loc().y())) {
      std::cout << "Initial state out of bounds\n";
      return unexpected<Error>{Error::OUT_OF_BOUNDS};
    }
    AStarPath path;
    State start = {initial};
    path.emplace_back(start);
    if (path.empty()) {
      std::cout << "Sum Ding Wong: Path empty" << std::endl;
      return unexpected<Error>{Error::INTERNAL_ERROR};
    }
    auto initial_cost                                     = Cost(0);
    path.f_score                                          = target->heuristic(path);
    path.g_score                                          = initial_cost;
    current_g_costs[initial.loc().y()][initial.loc().x()] = initial_cost;
    open_set.push(path);

    // A* loop
    while (!open_set.empty()) {
      // Choose path with lowest f score
      AStarPath current = open_set.top();
      open_set.pop();

      // Check if goal
      if (target->satisfiesCriteria(current)) {
        std::cout << "Planner finished, time: " << sw.elapsed().count() << " s" << std::endl;
        return current;
      }

      // Check time limit
      if (sw.elapsed().count() > time_limit) {
        std::cout << "Planner timeout, time: " << sw.elapsed().count() << " s" << std::endl;
        std::cout << target << std::endl;
        return current;
      }

      // Find neighbours
      const std::array<Vec2, 4> movement{Vec2(1, 0), Vec2(0, 1), Vec2(-1, 0), Vec2(0, -1)};
      State current_state = current.back();
      for (auto const &i : movement) {
        State neighbour;
        neighbour.setLoc(current_state.loc() + i);
        auto cost_to_neigbour = provider->costBetween(StateQuery(current_state, neighbour));
        // std::cout << "Neighbour: " << neighbour.loc().y() << " " << neighbour.loc().x() <<
        // std::endl;

        // If neighbour already on path, continue
        if (current.point_on_path(neighbour)) {
          // std::cout << "Point on path\n";
          continue;
        }
        // If out of bounds, continue
        if (provider->env.isOutOfBounds(neighbour.loc().x(), neighbour.loc().y())) {
          continue;
        }

        if (cost_to_neigbour.has_value()) {
          // std::cout << "Cost has value\n";
          if (cost_to_neigbour.value().type() != Cost::UNKNOWN) {
            // std::cout << "Cost is not unknown\n";

            // Generate new path
            AStarPath new_path = current;
            new_path.emplace_back(neighbour);
            new_path.g_score = new_path.g_score + cost_to_neigbour.value() +
                               agv_math::distanceBetween(current_state.loc(), neighbour.loc());

            new_path.f_score = new_path.g_score + target->heuristic(new_path);

            // New point reached
            if (current_g_costs[neighbour.loc().y()][neighbour.loc().x()].type() == Cost::UNKNOWN) {
              // std::cout << "New point\n";
              current_g_costs[neighbour.loc().y()][neighbour.loc().x()] = new_path.g_score;
              current_g_costs[neighbour.loc().y()][neighbour.loc().x()] =
                  Cost(new_path.g_score.value(), Cost::NORMAL);
              open_set.push(new_path);
              continue;
            }
            // Visited before, but new cost is lower
            else if (new_path.g_score < current_g_costs[neighbour.loc().y()][neighbour.loc().x()]) {
              // std::cout << "Old point, better\n";
              current_g_costs[neighbour.loc().y()][neighbour.loc().x()] = new_path.g_score;
              open_set.push(new_path);
            }
            // Visited before, new cost same or higher
            else {
              // std::cout << "Old point, worse\n";
              continue;
            }
          } else {
            std::cout << "Cost is unknown\n";
            // std::cout << "Current location:\n" << current_state.loc() << std::endl;
          }
        } else {
          std::cout << "Planner error, no cost value, time: " << sw.elapsed().count() << " s"
                    << std::endl;
          return unexpected<Error>{Error::INTERNAL_ERROR};
        }
      }
      closed_set.emplace_back(current_state);
    }
    std::cout << "Planner error, time: " << sw.elapsed().count() << " s" << std::endl;
    return unexpected<Error>{Error::INTERNAL_ERROR};
  } // end of plan

  IPlanner *setCostProvider(std::weak_ptr<ICostProvider> provider) override {
    cost_provider = provider;
    return this;
  }

  IPlanner *setTimeLimit(seconds_t limit) override {
    time_limit = limit;
    return this;
  }

private:
  std::priority_queue<AStarPath, std::vector<AStarPath>, AStarPathComparator> open_set;
  std::vector<State> closed_set;
  // Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic> current_costs;
  seconds_t time_limit = 1;
};

int main() {
  spdlog::stopwatch sw;
  std::cout << "======================\n";
  std::cout << "\033[0;33mTest application " << codejam22::app::project_version << "\033[0m\n";
  std::cout << "======================\n";
  spdlog::info("Planning library version {}", pathplanning::version());
  std::cout << "======================\n";

  // Cost provider and data
  auto cost_provider = std::make_shared<SimpleCostProvider>();
  SimpleEnvironment simple_env;
  ImportedData imported_data = import_from_csv();
  simple_env.env             = imported_data.cost_map;
  cost_provider->env         = simple_env;

  /*
  std::cout << "Map: " << cost_provider->env.env.cols() << " " << cost_provider->env.env.rows()
            << std::endl;

  std::cout << "Start\n"
            << imported_data.start << "\nFinish\n"
            << imported_data.finish << std::endl;
  */

  // Planner
  SimplePlanner planner;
  planner.setTimeLimit(120);
  planner.setCostProvider(std::weak_ptr<SimpleCostProvider>(cost_provider));

  // Initial
  State initial_state;
  initial_state.setStateElement(imported_data.start[0], 0);
  initial_state.setStateElement(imported_data.start[1], 1);

  // Target(s)
  TargetList targets;
  Vec2 target_loc(imported_data.finish[0], imported_data.finish[1]);
  PointTarget point_target(target_loc);
  State::Distance allowed_error;
  allowed_error.loc_distance = Vec2(0.5, 0.5);
  point_target.setAllowedError(allowed_error);
  targets.emplace_back(std::make_unique<PointTarget>(point_target));

  // Planner plans
  SimplePlanner::PathOrError path = planner.plan(initial_state, targets);

  // Output path and costmap to something
  auto map = cost_provider->env.env;
  if (path.has_value()) {
    // draw_path(map, path.value()); // In terminal
    export_to_csv(map, path.value());
  } else {
    std::cout << "Sum Ding Wong: path has no value"
              << "\n";
  }
  // when done, open up path in jupyter notebook
  spdlog::warn("Done {}", sw);
}