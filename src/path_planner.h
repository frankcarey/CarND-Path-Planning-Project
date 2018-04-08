//
// Created by Carey, Frank, Sr. on 3/3/18.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include "vehicle.h"

using namespace vehicle;


class PathPlanner {
public:
  PathPlanner();
  std::pair<fsm::STATE, vector<Vehicle>> choose_next_state(VehicleController &ctrl, map<int, vector<Vehicle>> &other_vehicle_predictions);
  double calculate_cost(VehicleController &ctrl, vector<Vehicle> &candidate_trajectory, std::map<int, vector<Vehicle>> &other_vehicle_predictions);

};
#endif //PATH_PLANNING_PATH_PLANNER_H
