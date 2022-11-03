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
    auto loc = wp.target.loc();
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
    auto loc = wp.target.loc();
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
struct SimpleEnvironment {
  // task1
  // create and store environment costmap
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

  // std::vector<State> get_neighbours(State current) {
  //   std::vector<State> neighbours;
  //   std::vector<Vec2> movement{Vec2(1, 0), Vec2(0, 1), Vec2(-1, 0), Vec2(0, -1)};

  //   for (auto const &i : movement) {
  //     State neighbour;
  //     neighbour.setLoc(current.loc() + i);
  //     if
  //       neighbours.emplace_back(neighbour);
  //   }

  //   return neighbours;
  // }
  SimpleEnvironment env;
};

struct AStarPath : Path {
  Cost f_score = Cost(std::numeric_limits<double>::max());
  Cost g_score = Cost(std::numeric_limits<double>::max());
  bool operator>(const AStarPath &p) const { return (f_score > p.f_score); }
  bool operator<(const AStarPath &p) const { return (f_score < p.f_score); }
  // std::vector<AStarPath> getNeighbours(std::weak_ptr<ICostProvider> provider) {
  //   std::vector<AStarPath> neighbours;

  //   return neighbours;
  // }
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
    auto provider      = std::static_pointer_cast<SimpleCostProvider>(cost_provider.lock());
    auto const &target = targets.at(0);

    AStarPath path;
    Waypoint start = {initial};
    path.emplace_back(start);
    path.f_score = target->heuristic(path);
    path.g_score = Cost(0);
    open_set.push(path);
    while (!open_set.empty()) {
      // Choose path with lowest f score
      AStarPath current = open_set.top();
      // std::cout << "Current g score: " << current.g_score;

      open_set.pop();
      // Check time limit
      if (sw.elapsed().count() > time_limit) {
        std::cout << "Planner timeout, time: " << sw.elapsed().count() << " s" << std::endl;
        return current;
      }
      // Check if goal
      if (target->satisfiesCriteria(current)) {
        std::cout << "Planner finished, time: " << sw.elapsed().count() << " s" << std::endl;
        return current;
      }

      // TODO break this into pieces
      const std::array<Vec2, 4> movement{Vec2(1, 0), Vec2(0, 1), Vec2(-1, 0), Vec2(0, -1)};
      State current_state = current.back().target;
      for (auto const &i : movement) {
        State neighbour;
        neighbour.setLoc(current_state.loc() + i);
        bool visited = false;
        for (auto i : closed_set) {
          if (i.loc() == neighbour.loc()) {
            // std::cout << "Been here, done that\n";
            visited = true;
          }
        }
        if (visited) {
          continue;
        }
        auto cost = provider->costBetween(StateQuery(current_state, neighbour));
        if (cost.has_value()) {
          if (cost.value().type() != Cost::UNKNOWN) {
            std::cout << "Loc: " << neighbour.loc().transpose() << "\n";
            std::cout << cost.value();
            AStarPath new_path = current;
            Waypoint wp;
            wp.target = neighbour;
            new_path.emplace_back(wp);
            std::cout << "Previous g_score: " << new_path.g_score;
            new_path.g_score = new_path.g_score + cost.value() +
                               agv_math::distanceBetween(current_state.loc(), neighbour.loc());

            std::cout << "New g_score: " << new_path.g_score;
            new_path.f_score = new_path.g_score;
            new_path.f_score = new_path.f_score + target->heuristic(new_path);
            std::cout << "New f_score: " << new_path.f_score << "\n";
            open_set.push(new_path);
          } else {
            std::cout << "Cost is unknown\n";
          }
        }
      }
      closed_set.emplace_back(current_state);
    }
    // for (auto const &target : targets) {
    //    // std::cout << target->heuristic(path);
    //  }
    std::cout << "Planner error, time: " << sw.elapsed().count() << " s" << std::endl;
    return unexpected<Error>{Error::INTERNAL_ERROR};
  }
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
  seconds_t time_limit = 1;
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
  auto cost_provider = std::make_shared<SimpleCostProvider>();
  SimpleEnvironment simple_env;
  ImportedData imported_data = import_from_csv();
  simple_env.env             = imported_data.cost_map;
  cost_provider->env         = simple_env;

  State initial_state;
  initial_state.setStateElement(imported_data.start[0], 0);
  initial_state.setStateElement(imported_data.start[1], 1);
  // std::cout << initial_state.getState() << "\n";

  // SimpleCostProvider::CostOrError cost = cost_provider->costBetween({initial_state,
  // goal_state}); if (cost.has_value()) {
  //   std::cout << cost.value() << "\n";
  // } else {
  //   std::cout << "Sum Ding Wong"
  //             << "\n";
  // }

  // task 2 - integrate this
  // create planner object
  // make planner plan from initial state to goal state
  // planner.plan(costProvider, initialState, targetState)
  SimplePlanner planner;
  planner.setTimeLimit(20);
  planner.setCostProvider(std::weak_ptr<SimpleCostProvider>(cost_provider));
  TargetList targets;
  Vec2 target_loc(imported_data.finish[0], imported_data.finish[1]);
  PointTarget point_target(target_loc);
  State::Distance allowed_error;
  allowed_error.loc_distance = Vec2(0.5, 0.5);
  point_target.setAllowedError(allowed_error);
  targets.emplace_back(std::make_unique<PointTarget>(point_target));
  SimplePlanner::PathOrError path = planner.plan(initial_state, targets);
  // task 3
  // output path and costmap to something
  // Make a boolean map marking the points which path visits
  auto map = cost_provider->env.env;
  if (path.has_value()) {
    // draw_path(map, path.value());
    export_to_csv(map, path.value());
  } else {
    std::cout << "Sum Ding Wong: path has no value"
              << "\n";
  }

  // task 4
  // when done, open up path in jupyter notebook
  // visualise results to validate behaviour

  spdlog::warn("Done {}", sw);
}