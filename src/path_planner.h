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
  vector<double> keepVelPoly(vector<double> conds, double T);
  vector<double> minJerkPoly(vector<double> conds, double T);
  vector<combiTraj> generate_trajectories(VehicleController carCtl, vector<double> conds_s, vector<double> conds_d,
                                          double time_horizon, double l_desired, vector< vector<double> > & near_cars);
  bool funcia (combiTraj i, combiTraj j);
};
#endif //PATH_PLANNING_PATH_PLANNER_H
