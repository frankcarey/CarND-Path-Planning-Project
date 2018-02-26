//
// Created by Carey, Frank, Sr. on 2/19/18.
//

#ifndef PATH_PLANNING_FSM_H
#define PATH_PLANNING_FSM_H

#include <vector>
#include "utils.h"
#include "vehicle.h"

using namespace std;
using namespace utils;
using namespace vehicle;

namespace fsm {
  enum STATE {
    CS = 0, // Constant speed,
    KL = 1, // 'Keep Lane',
    PLCL = 2, // 'Prepare Lane Change: Left'
    PLCR = 3, // 'Prepare Lane Change: Right'
    LCL = 4, // 'Lane Change: Left'
    LCR = 5, // 'Lane Change: Right'
  };

  map<STATE, int> LANE_DIRECTION;

  class VehicleFSM {
  public:
    STATE state;
    Map trackMap;

    VehicleFSM(STATE state, Map &trackMap);

    VehicleFSM();

    vector<STATE> successor_states(int lane);
    double calculate_cost(vector<Vehicle> &candidate_trajectory, std::map<int, vector<Vehicle>> &other_vehicle_predictions);
  };


};

#endif //PATH_PLANNING_FSM_H