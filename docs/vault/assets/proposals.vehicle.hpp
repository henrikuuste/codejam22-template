/*******************
 * @Proposal 1a
 *******************/
struct PathPlanner {
  // predictState should use vehicle physical model to predict the state at the next node depending
  // on current state and next nodes position physical model should be in a separate library (same
  // physical model for UKF and path planner etc in a separate library)
  // void setInitialState(StateT state);
  // void setGoal(PositionT postition);
  StateT predictState(StateT currentState, PositionT nextPosition);
};

/*******************
 * @Proposal 2a
 *******************/
// No vehicle concept

/*******************
 * @Proposal 3
 *******************/
// No vehicle concept

/*******************
 * @Proposal 4
 *******************/
struct machineConfig {
  Eigen::Vector2d machine_dimensions;
};

// NOTE this is provided to the planner, but not to the cost map

/*******************
 * @Proposal 5
 *******************/
// No vehicle concept

/*******************
 * @Proposal 6
 *******************/
using Size = Eigen::Vector3f;

struct Vehicle {
  Size dimensions; // assume constant symmetrical box shaped vehicle
  float minimum_turn_radius;
};

struct Settings {
  Vehicle vehicle;
  // ...
};

// NOTE this is provided to the planner, but not to the cost map

/*******************
 * @Proposal 7
 *******************/
class ICostMap {
protected:
  Eigen::Vector3d vehicle_dim_;

public:
  virtual void setVehicleDimensions(double x, double y, double z) = 0;
};

/*******************
 * @Proposal 8
 *******************/
// No vehicle concept