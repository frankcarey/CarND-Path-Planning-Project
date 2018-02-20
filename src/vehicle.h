#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  double preferred_buffer = 6; // impacts "keep lane" behavior.

  double s;

  double d;

  double v;

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
  Vehicle();

  Vehicle(double s, double d, double v, double a=0, double yaw=0, string state="CS");

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions, std::function<double(Vehicle, map<int, vector<Vehicle>>, vector<Vehicle>)> calculate_cost);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void increment(int dt);

  double position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  vector<Vehicle> generate_predictions(int horizon=2);

  void realize_next_state(vector<Vehicle> trajectory);

  void configure(vector<double> road_data);

  int get_lane();

  bool in_my_lane(Vehicle &other);

  double static lane_to_d(double lane);

  int static d_to_lane(double d);

  double distance_from_me(Vehicle &other, double time_delta = 0);

  double accelerate(double factor=0.01);

  double decelerate(double factor=0.01);

  Vehicle clone();

};

#endif // VEHICLE_H