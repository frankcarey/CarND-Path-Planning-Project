//
// Created by Carey, Frank, Sr. on 3/3/18.
//

#include "path_planner.h"

using namespace vehicle;


std::pair<fsm::STATE, vector<Vehicle>> PathPlanner::choose_next_state(VehicleController ctrl, map<int, vector<Vehicle>> &other_vehicle_predictions) {
  /*
  Here you can implement the transition_function code from the Behavior Planning Pseudocode
  classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
  to the next state.

  INPUT: A predictions map. This is a map of vehicle id keys with predicted
      vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
      the vehicle at the current timestep and one timestep in the future.
  OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

  */
  vector<fsm::STATE> states = ctrl.fsm->successor_states(ctrl.get_lane());
  double cost;
  vector<double> costs;
  vector<fsm::STATE> final_states;
  vector<vector<Vehicle>> final_trajectories;

  for (fsm::STATE &state : states) {
    vector<Vehicle> candidate_trajectory = ctrl.generate_trajectory(state, other_vehicle_predictions);
    if (!candidate_trajectory.empty()) {
      cost = ctrl.calculate_cost(candidate_trajectory, other_vehicle_predictions);
      costs.push_back(cost);
      final_trajectories.push_back(candidate_trajectory);
      final_states.push_back(state);
    }
  }

  auto best_cost = min_element(begin(costs), end(costs));
  auto best_idx = (int) distance(begin(costs), best_cost);

  return {final_states[best_idx], final_trajectories[best_idx]};
}


double PathPlanner::calculate_cost(vector<Vehicle> &candidate_trajectory, std::map<int, vector<Vehicle>> &other_vehicle_predictions) {
  /*
  Sum weighted cost functions to get total cost for trajectory.
  */
  double cost = 0.0;

//    //Add additional cost functions here.
//    vector<function<double(const VehicleController &, const vector<VehicleController> &,
//                           const map<int, vector<VehicleController>> &, map<string, double> &)>> cf_list = {
//        goal_distance_cost, inefficiency_cost};
//    vector<double> weight_list = {REACH_GOAL, EFFICIENCY};
//
//    for (int i = 0; i < cf_list.size(); i++) {
//      double new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, predictions, trajectory_data);
//      cost += new_cost;
//    }

  return cost;
}

