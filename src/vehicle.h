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
  private:
    Position _position;
    double _a;
    double _v;
    double _yaw_delta;
    double _dx;
    double _dy;

  public:

    Vehicle();

    explicit Vehicle(Position position);

    double v();
    void v(double v);
    double a();
    void a(double a);
    double x();
    void x(double x);
    double y();
    void y(double y);
    double yaw();
    void yaw(double yaw);
    double yaw_delta();
    void yaw_delta(double yaw_delta);
    Position position();
    void position(Position pos);

    Vehicle clone();

  };


  class VehicleController {
  public:

    struct collider {

      bool collision; // is there a collision?
      int time; // time collision happens

    };

    double preferred_buffer = 6; // impacts "keep lane" behavior.

    Vehicle vehicle;

    Map trackMap;

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

    VehicleController(Vehicle &v, fsm::VehicleFSM &fsm, Map &trackMap);

    vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &other_vehicle_predictions);

    vector<Vehicle> generate_trajectory(fsm::STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions);

    Vehicle get_lane_kinematic(int lane, map<int, vector<Vehicle>> &other_vehicle_predictions);

    int get_vehicle_behind(int lane, map<int, vector<Vehicle>> &other_vehicle_predictions);

    int get_vehicle_ahead(int lane, map<int, vector<Vehicle>> &other_vehicle_predictions);

    vector<Vehicle> constant_speed_trajectory(map<int, vector<Vehicle>> &other_vehicle_predictions);

    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &other_vehicle_predictions);

    vector<Vehicle> lane_change_trajectory(fsm::STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions);

    vector<Vehicle> prep_lane_change_trajectory(fsm::STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions);

    void increment(int dt);

    Position position_at(int t);



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

  };
}
#endif // VEHICLE_H