//
// Created by Carey, Frank, Sr. on 2/19/18.
//

#ifndef PATH_PLANNING_FSM_H
#define PATH_PLANNING_FSM_H

#include <vector>
#include <map>
#include "utils.h"

using namespace std;
using namespace utils;

namespace fsm {
  enum STATE {
    CS = 0, // Constant speed,
    KL = 1, // 'Keep Lane',
    PLCL = 2, // 'Prepare Lane Change: Left'
    PLCR = 3, // 'Prepare Lane Change: Right'
    LCL = 4, // 'Lane Change: Left'
    LCR = 5, // 'Lane Change: Right'
  };

  const map<STATE, int> LANE_DIRECTION = {
      {STATE::PLCL, 1},
      {STATE::LCL,  1},
      {STATE::LCR,  -1},
      {STATE::PLCR, -1}
  };

  class VehicleFSM {
  public:
    STATE state;
    Map trackMap;

    VehicleFSM(STATE state, Map &trackMap);

    VehicleFSM();

    vector<STATE> successor_states(int lane);
  };


};

#endif //PATH_PLANNING_FSM_H