
#include <algorithm>
#include <iostream>
#include "vehicle.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle() = default;

Vehicle::Vehicle(float s, float d, float v, float a, float yaw, string state) {

  this->s = s;
  this->d = d;
  this->v = v;
  this->a = a;
  this->yaw = yaw;
  this->state = state;
  //max_acceleration = -1;

}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions,
                                           std::function<float(Vehicle, map<int, vector<Vehicle>>, vector<Vehicle>)> calculate_cost) {
  /*
  Here you can implement the transition_function code from the Behavior Planning Pseudocode
  classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
  to the next state.

  INPUT: A predictions map. This is a map of vehicle id keys with predicted
      vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
      the vehicle at the current timestep and one timestep in the future.
  OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

  */
  vector<string> states = successor_states();
  float cost;
  vector<float> costs;
  vector<string> final_states;
  vector<vector<Vehicle>> final_trajectories;

  for (auto &state : states) {
    vector<Vehicle> trajectory = generate_trajectory(state, predictions);
    if (! trajectory.empty()) {
      cost = calculate_cost(*this, predictions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }

  auto best_cost = min_element(begin(costs), end(costs));
  auto best_idx = (int) distance(begin(costs), best_cost);
  return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
  /*
  Provides the possible next states given the current state for the FSM
  discussed in the course, with the exception that lane changes happen
  instantaneously, so LCL and LCR can only transition back to KL.
  */
  vector<string> states;
  states.emplace_back("KL");
  string state = this->state;
  if(state == "KL") {
    states.emplace_back("PLCL");
    states.emplace_back("PLCR");
  } else if (state == "PLCL") {
    if (this->get_lane() != lanes_available - 1) {
      states.emplace_back("PLCL");
      states.emplace_back("LCL");
    }
  } else if (state == "PLCR") {
    if (this->get_lane() != 0) {
      states.emplace_back("PLCR");
      states.emplace_back("LCR");
    }
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
  /*
  Given a possible next state, generate the appropriate trajectory to realize the next state.
  */
  vector<Vehicle> trajectory;
  if (state == "CS") {
    trajectory = constant_speed_trajectory();
  } else if (state== "KL") {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state == "LCL" || state == "LCR") {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state == "PLCL"|| state == "PLCR") {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }
  return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
  /*
  Gets next timestep kinematics (position, velocity, acceleration)
  for a given lane. Tries to choose the maximum velocity and acceleration,
  given other vehicle positions and accel/velocity constraints.
  */
  float max_velocity_accel_limit = this->max_acceleration + this->v;
  float new_position;
  float new_velocity;
  float new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

    if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
      new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
    } else {
      auto max_velocity_in_front = (float) ((vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a));
      new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
    }
  } else {
    new_velocity = min(max_velocity_accel_limit, this->target_speed);
  }

  new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
  new_position = this->s + new_velocity + new_accel / (float) 2.0;
  return{new_position, new_velocity, new_accel};

}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  /*
  Generate a constant speed trajectory.
  */
  float next_pos = position_at(1);
  vector<Vehicle> trajectory = {Vehicle(this->s, this->d, this->v, this->a, 0, this->state),
                                Vehicle(next_pos, this->d, this->v, 0, 0, this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
  /*
  Generate a keep lane trajectory.
  */
  vector<Vehicle> trajectory = {Vehicle(this->s, this->d, this->v, this->a, 0, state)};
  vector<float> kinematics = get_kinematics(predictions, (int) round(this->get_lane()) );
  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];
  trajectory.emplace_back(new_s, this->d, new_v, new_a, 0, "KL");
  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
  /*
  Generate a trajectory preparing for a lane change.
  */
  float new_s;
  float new_v;
  float new_a;
  Vehicle vehicle_behind;
  int new_lane = this->get_lane() + lane_direction[state];
  vector<Vehicle> trajectory = {Vehicle(this->s, this->d, this->v, this->a, 0, this->state)};
  vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->get_lane());

  if (get_vehicle_behind(predictions, this->get_lane(), vehicle_behind)) {
    //Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];

  } else {
    vector<float> best_kinematics;
    vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
    //Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  trajectory.emplace_back(new_s, this->d, new_v, new_a, 0, state);
  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
  /*
  Generate a lane change trajectory.
  */
  int new_lane = this->get_lane() + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  //Check if a lane change is possible (check if another vehicle occupies that spot).
  for (auto &prediction : predictions) {
    next_lane_vehicle = prediction.second[0];
    if (next_lane_vehicle.s == this->s && next_lane_vehicle.get_lane() == new_lane) {
      //If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  trajectory.emplace_back(this->s, this->d, this->v, this->a, 0, this->state);
  vector<float> kinematics = get_kinematics(predictions, new_lane);
  trajectory.emplace_back(kinematics[0], this->d, kinematics[1], kinematics[2], 0, state);
  return trajectory;
}

void Vehicle::increment(int dt = 1) {
  this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
  return this->s + this->v*t + this->a*t*t/(float)2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> other_cars, int lane, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
  */
  float max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (auto &it : other_cars) {
    // FYI: for a map<first, second>, it->second[0] would be the first instance of the second type (so the first vehicle).
    Vehicle other_car = it.second[0];

    if (temp_vehicle.get_lane() == this->get_lane() && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
  */
  float min_s = this->target_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (auto &prediction : predictions) {
    // FYI: for a map<first, second>, it->second[0] would be the first instance of the second type (so the first vehicle).
    temp_vehicle = prediction.second[0];
    if (temp_vehicle.get_lane() == this->get_lane() && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
  /*
  Generates predictions for non-ego vehicles to be used
  in trajectory generation for the ego vehicle.
  */
  vector<Vehicle> predictions;
  for(int i = 0; i < horizon; i++) {
    float next_s = position_at(i);
    float next_v = 0;
    if (i < horizon-1) {
      next_v = position_at(i+1) - s;
    }
    predictions.emplace_back(this->get_lane(), next_s, next_v, 0);
  }
  return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
  /*
  Sets state and kinematics for ego vehicle using the last state of the trajectory.
  */
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->target_lane = next_state.get_lane();
  //this->s = next_state.s;
  this->target_speed = next_state.v;
  //this->a = next_state.a;
}

void Vehicle::configure(vector<float> road_data) {
  /*
  Called by simulator before simulation begins. Sets various
  parameters which will impact the ego vehicle.
  */
  target_speed = (float) road_data[0];
  lanes_available = (int) road_data[1];
  target_s = (float) road_data[2];
  target_lane = (int) road_data[3];
  max_acceleration = (float) road_data[4];
  max_legal_speed = (float) road_data[5];
}

int Vehicle::get_lane() {
  // Lanes are 4m wide, so first divide and then subtract 1/2 a lane to center.
  // assumes 0 is the left most lane.
  return Vehicle::d_to_lane(this->d);
}

float Vehicle::lane_to_d(float lane) {
  // Lanes are 4m wide, so first multiply by 4 and then add 2 to center it.
  // assumes 0 is the left most lane.
  return lane*4 + 2;
}

int Vehicle::d_to_lane(float d) {
  // Lanes are 4m wide, so first multiply by 4 and then add 2 to center it.
  // assumes 0 is the left most lane.
  return (int) round(d / 4 - 0.5);
}

bool Vehicle::in_my_lane(Vehicle &other) {
  //const float lane_buffer = 0.5;
  int other_lane = other.get_lane();
  if (other_lane < 0) return false;
  int my_lane = this->get_lane();

  bool in_my_lane = (my_lane == other_lane);
  return in_my_lane;
  ;
}

float Vehicle::distance_from_me(Vehicle &other, float time_delta) {
  float other_car_s = other.s;
  other_car_s += (time_delta *  other.v); //if using time_delta, project car's s value out in time.
  return (other_car_s - this->s);
}


float Vehicle::accelerate(float factor) {
  float new_a = this->a + factor;
  if (abs(new_a) <= this->max_acceleration) {
    this->a = new_a;
  }
  return new_a;
}

float Vehicle::decelerate(float factor) {
  float new_a = this->a - factor;
  if (abs(new_a) <= this->max_acceleration) {
    this->a = new_a;
  }
  return new_a;
}

Vehicle Vehicle::clone() {
  return Vehicle(this->s, this->d, this->v, this->a, this->yaw, this->state);
}





