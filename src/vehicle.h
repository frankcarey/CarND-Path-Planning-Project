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

  float preferred_buffer = 6; // impacts "keep lane" behavior.

  float s;

  float d;

  float v;

  float a;

  float yaw;

  float target_speed;

  int target_lane;

  float target_s;

  int lanes_available;

  float max_acceleration;

  float max_legal_speed;

  string state;

  /**
  * Constructor
  */
  Vehicle();

  Vehicle(float s, float d, float v, float a=0, float yaw=0, string state="CS");

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions, std::function<float(Vehicle, map<int, vector<Vehicle>>, vector<Vehicle>)> calculate_cost);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  vector<Vehicle> generate_predictions(int horizon=2);

  void realize_next_state(vector<Vehicle> trajectory);

  void configure(vector<float> road_data);

  int get_lane();

  bool in_my_lane(Vehicle &other);

  float static lane_to_d(float lane);

  int static d_to_lane(float d);

  float distance_from_me(Vehicle &other, float time_delta = 0);

  float accelerate(float factor=0.01);

  float decelerate(float factor=0.01);



};

#endif // VEHICLE_H