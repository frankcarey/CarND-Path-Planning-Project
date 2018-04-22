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
  double calculate_cost(double speed_limit, Trajectory2D &trajectory);
  vector<double> sPoly(vector<double> conds, double T);
  vector<double> dPoly(vector<double> conds, double T);
  vector<Trajectory2D> generate_trajectories(VehicleController carCtl, vector<double> conds_s, vector<double> conds_d,
                                          double time_horizon, vector< vector<double> > & near_cars);

  int minimal_cost_trajectory(vector<Trajectory2D> combSet);
};
#endif //PATH_PLANNING_PATH_PLANNER_H
