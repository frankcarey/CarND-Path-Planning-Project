//
// Created by Carey, Frank, Sr. on 2/19/18.
//
#include "fsm.h"
#include <vector>

#include "utils.h"

using namespace std;
using namespace utils;

namespace fsm {

  VehicleFSM::VehicleFSM(STATE state, Map *trackMap) {
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
    if (this->trackMap->is_lane_available(lane-1)) {
      states.emplace_back(LCL);
    }
    if (this->trackMap->is_lane_available(lane+1)) {
      states.emplace_back(LCR);
    }
    return states;
  }

  const double REACH_GOAL = pow(10, 6);
  const double EFFICIENCY = pow(10, 5);

  void add_cost_fn(){

  } //TODO: Add ability to add cost functions to the FSM? (or is this overthinking it?)
}