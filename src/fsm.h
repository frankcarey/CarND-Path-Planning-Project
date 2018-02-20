//
// Created by Carey, Frank, Sr. on 2/19/18.
//

#ifndef PATH_PLANNING_FSM_H
#define PATH_PLANNING_FSM_H

#include <vector>
#include "utils.h"

using namespace std;
using namespace utils;

namespace fsm {
  enum STATE {
    KL = 0, // 'Keep Lane',
    PLCL = 1, // 'Prepare Lane Change: Left'
    PLCR = 2, // 'Prepare Lane Change: Right'
    LCL = 3, // 'Lane Change: Left'
    LCR = 4, // 'Lane Change: Right'
  };


  class VehicleFSM {
  public:
    STATE state;
    Map map;

    VehicleFSM(STATE state, Map &map);

    VehicleFSM();

    vector<STATE> successor_states(int lane);
    double calculate_cost();
  };


};

#endif //PATH_PLANNING_FSM_H