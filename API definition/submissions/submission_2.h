#include <iostream>
#include <utility>
#include <vector>
namespace cjpp /* CodeJam PathPlanner */
{
using MapIndex = std::pair<int, int>;
// Assume 2D index to map, x and y

/**
 * This class is a base class for any kind of path planner algorithm. It has one function for
 * solving a map provided It returns a result structure.
 *
 * TODO: (Or question) See if having a virtual function is a good idea. This may not me a generic
 * idea for different algorithms.
 */
class basePlanner {
public:
  struct pathPlannerResult {
    MapIndex startPoint;
    MapIndex endPoint;
    bool solved;
    // Result is binary - failed or succeeded
    // We get no information on reason for failure
    std::vector<std::vector<MapIndex>> paths;
    // Multiple paths
  };

  /**
   * @brief
   *
   * @param map A cost map generated by mapGenerator class
   * @param startPoint Starting point of the vehicle
   * @param endPoint Destination point of the vehicle
   * @param obstacleCost A fast way of testing some costs as obstacles without changing/regenerating
   * the cost map
   * @param nPath How many path solutions we want
   * @return pPR Returns a class with success status, solved path(s), start and end points.
   */
  virtual pathPlannerResult solver(const mapGenerator::Map &map, MapIndex startPoint,
                                   MapIndex endPoint, int obstacleCost = 0,
                                   std::size_t nPath = 1) = 0;
  // Cost map input - assuming static cost
  // Single goal/waypoint planner
  // Stateful planner (class)
  // Integer obstacle cost - can be negative. No semantic meaning at this point
  // Threshold value for what is considered an obstacle (not sure if >= or <=)
  // Can generate multiple paths
  // Assuming a discretized map - indices
  // No assumption on the vehicle
  // No assumption on heuristic
  //   How do we measure distance?
};

/**
 * MapGenerator
 * This class can generate five different cost maps from a given map format
 *
 */

class mapGenerator {
public:
  struct Map {
    std::size_t mapWidth;
    std::size_t mapHeigth;
    /*
    Add map here, vector, array, matrix, etc
    */
  };
  struct MapForController;

  // TODO add a map parameter for the original map, probably a matrix

  Map simpleMap(std::size_t maxCost, std::size_t minCost = 0);
  Map simpleMapWithSafeZone(std::size_t maxCost, std::size_t safetyRange, std::size_t minCost = 0);
  Map vehicleMap(std::size_t vehicleWidth, std::size_t vehicleHeigth, std::size_t offsetw,
                 std::size_t offseth, std::size_t maxCost, std::size_t minCost = 0);
  // Vehicle dimensions are baked into static cost
  // Assuming that vehicle is axially symmetric - orientation does not affect dimensions
  //   Width and height are used to calculate the circle
  //   Cost map calculator is responsible for vehicle size and safety
  // Unsafe implementations of cost are possible

  /**
   * @brief
   *
   * @param vehicleWidth vehicle size - width
   * @param vehicleHeigth vehicle size - height
   * @param offsetw vehicle size offset - width, use this value to expend coliding area around the
   * vehicle
   * @param offseth vehicle size offset - heigth, use this value to expend coliding area around the
   * vehicle
   * @param maxCost Upper threshold value for obstacles. What is above this threshold is an obstacle
   * @param safetyRange Increased cost areas around obstacles
   * @param minCost Lower threshold value for obstacles. What is below this threshold is an obstacle
   * @return map
   */Map vehicleMapWithSafeZone(std::size_t vehicleWidth, std::size_t vehicleHeigth,
                             std::size_t offsetw, std::size_t offseth, std::size_t maxCost,
                             std::size_t safetyRange, std::size_t minCost = 0);
  MapForController narrowPathMap();
};

} // namespace cjpp
