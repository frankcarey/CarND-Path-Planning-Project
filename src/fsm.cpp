//
// Created by Carey, Frank, Sr. on 2/19/18.
//
#include "fsm.h"
#include <vector>

#include "utils.h"

using namespace std;
using namespace utils;

namespace fsm {

  VehicleFSM::VehicleFSM() {
    this->state = STATE::KL;
  };

  VehicleFSM::VehicleFSM(STATE state, Map &trackMap): trackMap(trackMap) {
    this->state = state;
  }

  vector<STATE> VehicleFSM::successor_states(int lane) {
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector <STATE> states;

    states.emplace_back(KL);
    return states;
    //TODO: Fix this.
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

  void add_cost_fn(){

  } //TODO: Add ability to add cost functions to the FSM? (or is this overthinking it?)
}