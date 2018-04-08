//
// Created by Carey, Frank, Sr. on 3/3/18.
//

#include "path_planner.h"

using namespace vehicle;

PathPlanner::PathPlanner(){
}

std::pair<fsm::STATE, vector<Vehicle>> PathPlanner::choose_next_state(VehicleController &ctrl, map<int, vector<Vehicle>> &other_vehicle_predictions) {
  /*
  Here you can implement the transition_function code from the Behavior Planning Pseudocode
  classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
  to the next state.

  INPUT: A predictions map. This is a map of vehicle id keys with predicted
      vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
      the vehicle at the current timestep and one timestep in the future.
  OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

  */
  int curr_lane = ctrl.get_lane();
  vector<fsm::STATE> states = ctrl.fsm->successor_states(curr_lane);
  double cost;
  vector<double> costs;
  vector<fsm::STATE> final_states;
  vector<vector<Vehicle>> final_trajectories;

  for (fsm::STATE &state : states) {
    vector<Vehicle> candidate_trajectory = ctrl.generate_trajectory(state, other_vehicle_predictions);
    if (!candidate_trajectory.empty()) {
      cout << "state cost for: " << state << "\n";
      cost = this->calculate_cost(ctrl, candidate_trajectory, other_vehicle_predictions);
      costs.push_back(cost);
      final_trajectories.push_back(candidate_trajectory);
      final_states.push_back(state);
    }
  }

  auto best_cost = min_element(begin(costs), end(costs));
  auto best_idx = (int) distance(begin(costs), best_cost);

  return {final_states[best_idx], final_trajectories[best_idx]};
}


double PathPlanner::calculate_cost(VehicleController &ctrl, vector<Vehicle> &candidate_trajectory, std::map<int, vector<Vehicle>> &other_vehicle_predictions) {
  /*
  Sum weighted cost functions to get total cost for trajectory.
  */
  double cost = 0.0;

  cost += fabs(ctrl.trackMap->speed_limit() - candidate_trajectory.back().v());

  // Make it change lanes..
  int lane = ctrl.trackMap->getXYLane(candidate_trajectory.back().position());
  cout << "lane: " << lane << " \n";
  int car_ahead_id = ctrl.get_vehicle_ahead(lane, other_vehicle_predictions);
  if(car_ahead_id > 0) {
    Vehicle car_ahead = other_vehicle_predictions[car_ahead_id].back();
    cost += fabs(ctrl.trackMap->speed_limit() - car_ahead.v());
  }

  cout << "\n";
  cout << "cost: " << cost << "\n";
  cout << "\n";
  return cost;
}

