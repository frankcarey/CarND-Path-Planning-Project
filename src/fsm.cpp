//
// Created by Carey, Frank, Sr. on 2/19/18.
//
#include "fsm.h"
#include <vector>

#include "utils.h"
#include "vehicle.h"

using namespace std;
using namespace utils;
using namespace vehicle;

namespace fsm {

  map<STATE, int> lane_direction STATE::

  VehicleFSM::VehicleFSM() {
    this->state = STATE::KL;
  };

  VehicleFSM::VehicleFSM(STATE state, Map &trackMap) {
    this->state = state;
    this->trackMap = trackMap;
  }

  vector<STATE> VehicleFSM::successor_states(int lane) {
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector <STATE> states;
    states.emplace_back(KL);
    if (state == KL) {
      states.emplace_back(PLCL);
      states.emplace_back(PLCR);
    } else if (state == PLCL) {
      if (this->trackMap.is_lane_available(lane-1)) {
        states.emplace_back(PLCL);
        states.emplace_back(LCL);
      }
    } else if (state == PLCR) {
      if (this->trackMap.is_lane_available(lane+1)) {
        states.emplace_back(PLCR);
        states.emplace_back(LCR);
      }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
  }

  const double REACH_GOAL = pow(10, 6);
  const double EFFICIENCY = pow(10, 5);

  void add_cost_fn(){} //TODO: Add ability to add cost functions to the FSM? (or is this overthinking it?)

  double VehicleFSM::calculate_cost(vector<Vehicle> &candidate_trajectory, std::map<int, vector<Vehicle>> &other_vehicle_predictions) {
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
}