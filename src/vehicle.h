#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "utils.h"
#include "fsm.h"

using namespace std;
using namespace utils;

namespace vehicle {
  class Vehicle {
  public:

    Vehicle();

    explicit Vehicle(Point pt);

    double x();

    double y();

    double yaw();

    double v();


    Point location;

    double dx;
    double dy;


  };


  class VehicleController {
  public:

    VehicleController(Vehicle &vehicle, fsm::VehicleFSM fsm, Map &map);

    map<string, int> lane_direction = {{"PLCL", 1},
                                       {"LCL",  1},
                                       {"LCR",  -1},
                                       {"PLCR", -1}};

    struct collider {

      bool collision; // is there a collision?
      int time; // time collision happens

    };

    double preferred_buffer = 6; // impacts "keep lane" behavior.

    Vehicle vehicle;

    Map map;

    fsm::VehicleFSM fsm;

    double a;

    double yaw;

    double target_speed;

    int target_lane;

    double target_s;

    int lanes_available;

    double max_acceleration;

    double max_legal_speed;

    string state;

    /**
    * Constructor
    */

    VehicleController(Vehicle v, fsm::VehicleFSM fsm, Map &map);

    vector<VehicleController> choose_next_state(std::map<int, vector<VehicleController>> predictions,
                                                std::function<double(VehicleController,
                                                                     std::map<int, vector<VehicleController>>,
                                                                     vector<VehicleController>)> calculate_cost);

    vector<string> successor_states();

    vector<VehicleController> generate_trajectory(string state, map<int, vector<VehicleController>> predictions);

    vector<double> get_kinematics(map<int, vector<VehicleController>> predictions, int lane);

    vector<VehicleController> constant_speed_trajectory();

    vector<VehicleController> keep_lane_trajectory(map<int, vector<VehicleController>> predictions);

    vector<VehicleController> lane_change_trajectory(string state, map<int, vector<VehicleController>> predictions);

    vector<VehicleController>
    prep_lane_change_trajectory(string state, map<int, vector<VehicleController>> predictions);

    void increment(int dt);

    double position_at(int t);

    bool get_vehicle_behind(map<int, vector<VehicleController>> predictions, int lane, VehicleController &rVehicle);

    bool get_vehicle_ahead(map<int, vector<VehicleController>> predictions, int lane, VehicleController &rVehicle);

    vector<VehicleController> generate_predictions(int horizon = 2);

    void realize_next_state(vector<VehicleController> trajectory);

    void configure(vector<double> road_data);

    int get_lane();

    bool in_my_lane(VehicleController &other);

    double static lane_to_d(double lane);

    int static d_to_lane(double d);

    double distance_from_me(VehicleController &other, double time_delta = 0);

    double accelerate(double factor = 0.01);

    double decelerate(double factor = 0.01);

    VehicleController clone();

  };
}
#endif // VEHICLE_H