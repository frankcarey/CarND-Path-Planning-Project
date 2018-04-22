#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <deque>
#include <string>
#include <chrono>

#include "utils.h"
#include "fsm.h"

using namespace std;
using namespace utils;
using namespace std::chrono;

namespace vehicle {


  class Vehicle {

  private:
    int _id;
    FrenetPos _fpos;
    double _a;
    double _v;
    double _yaw;
    double _yaw_delta;
    time_point<system_clock> _time;

  public:

    Vehicle();

    explicit Vehicle(int id, FrenetPos position);

    int id();

    void id(int id);

    time_point<system_clock> time();

    void time(time_point<system_clock> time);

    void addSeconds(double seconds);

    double secondsDiff(time_point<system_clock> other_time);

    double v();

    void v(double v);

    double a();

    void a(double a);

    double s();

    void s(double s);

    double d();

    void d(double s);

    double yaw();

    void yaw(double yaw);

    double yaw_delta();

    void yaw_delta(double yaw_delta);

    FrenetPos position();

    void position(FrenetPos pos);

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

    Vehicle last_path_vehicle;

    Map *trackMap;

    fsm::VehicleFSM *fsm;

    double max_acceleration;

    time_point<system_clock> last_update_time;

    double last_update_time_delta;

    std::deque<Vehicle> trajectory;

    /**
    * Constructor
    */

    VehicleController(Vehicle v, fsm::VehicleFSM *fsm, Map *trackMap);

    vector<Vehicle> generate_trajectory(fsm::STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions);

    Vehicle get_lane_kinematic(Vehicle vehicle, int lane, double timedelta, double target_velocity, map<int, vector<Vehicle>> &other_vehicle_predictions);

    int get_vehicle_behind(int lane, map<int, vector<Vehicle>> &other_vehicle_predictions);

    int get_vehicle_ahead(int lane, map<int, vector<Vehicle>> &other_vehicle_predictions);

    bool lane_opening_exists(int lane, map<int, vector<Vehicle>> &other_vehicle_predictions);

    vector<Vehicle> constant_speed_trajectory(map<int, vector<Vehicle>> &other_vehicle_predictions);

    vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &other_vehicle_predictions);

    vector<Vehicle> lane_change_trajectory(fsm::STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions);

    vector<Vehicle> prep_lane_change_trajectory(fsm::STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions);

    Vehicle predict_next(Vehicle &car, double timedelta);

    vector<Vehicle> generate_predictions(int n_steps, Vehicle other_car);

    int get_lane();

    double time_delta();

    void update(double x, double y, double yaw, double speed_mph, int prev_trajectory_size);

    void trim_prev_trajectory(int prev_size);

    void extend_trajectory(vector<Vehicle> &path);

    VehicleController clone();

  };
}

#endif // VEHICLE_H