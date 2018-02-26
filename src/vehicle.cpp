#include <algorithm>
#include <iostream>
#include <vector>
#include <map>
#include "vehicle.h"
#include "fsm.h"
#include "utils.h"

using namespace utils;
using namespace std;
using namespace fsm;

namespace vehicle {

/**
 * Initializes Vehicle
 */

  Vehicle::Vehicle()=default;

  Vehicle::Vehicle(Position pos) {
    this->_position = pos;
  }

  double Vehicle::x() {
    return this->_position.x;
  }

  void Vehicle::x(double x) {
    this->_position.x = x;
  }

  double Vehicle::y() {
    return this->_position.y;
  }

  void Vehicle::y(double y) {
    this->_position.y = y;
  }

  double Vehicle::yaw() {
    return this->_position.yaw;
  }

  void Vehicle::yaw(double yaw) {
    this->_position.yaw = yaw;
  }

  double Vehicle::v() {
    //TODO: should I use this or just use _v?
    //    return sqrt(this->_dx * this->_dx + this->_dy * this->_dy);
    return this->_v;
  }

  void Vehicle::v(double v) {
    this->_v = _v;
  }

  double Vehicle::a() {
    return this->_a;
  }

  void Vehicle::a(double a) {
    this->_a = a;
  }

  double Vehicle::yaw_delta() {
    // TODO: Get the actual value.
    return this->_yaw_delta;
  }

  void Vehicle::yaw_delta(double yaw_delta) {
    this->_yaw_delta = yaw_delta;
  }

  Position Vehicle::position() {
    // TODO: Get the actual value.
    return this->_position;
  }

  void Vehicle::position(Position pos) {
    this->_position = pos;
  }

  Vehicle Vehicle::clone() {

    Vehicle new_vehicle = Vehicle(this->position().clone());
    new_vehicle.v(this->v());
    new_vehicle.a(this->a());
    new_vehicle.yaw_delta(this->yaw_delta());
    return new_vehicle;
  }


/**
 * Initializes VehicleController
 */

  VehicleController::VehicleController(Vehicle &vehicle, VehicleFSM &fsm, Map &trackmap) {
    this->vehicle = vehicle;
    this->fsm = fsm;
    this->trackMap = trackMap;
  }

  vector<Vehicle> VehicleController::choose_next_state(map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    //vector<string> states = this->fsm->successor_states();
    vector<fsm::STATE> states = this->fsm.successor_states(this->get_lane());
    double cost;
    vector<double> costs;
    vector<fsm::STATE> final_states;
    vector<vector<Vehicle>> final_trajectories;

    for (fsm::STATE &state : states) {
      vector<Vehicle> candidate_trajectory = this->generate_trajectory(state, other_vehicle_predictions);
      if (!candidate_trajectory.empty()) {
        cost = this->fsm.calculate_cost(candidate_trajectory, other_vehicle_predictions);
        costs.push_back(cost);
        final_trajectories.push_back(candidate_trajectory);
      }
    }

    auto best_cost = min_element(begin(costs), end(costs));
    auto best_idx = (int) distance(begin(costs), best_cost);
    return final_trajectories[best_idx];
  }

  vector<Vehicle> VehicleController::generate_trajectory(fsm::STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state == fsm::STATE::CS) {
      trajectory = this->constant_speed_trajectory(other_vehicle_predictions);
    } else if (state == fsm::STATE::KL) {
      trajectory = this->keep_lane_trajectory(other_vehicle_predictions);
    } else if (state == fsm::STATE::LCL || state == fsm::STATE::LCR) {
      trajectory = this->lane_change_trajectory(state, other_vehicle_predictions);
    } else if (state == fsm::STATE::PLCL || state == fsm::STATE::PLCR) {
      trajectory = this->prep_lane_change_trajectory(state, other_vehicle_predictions);
    }
    return trajectory;
  }

  Vehicle VehicleController::get_lane_kinematic(int lane, map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    double timedelta = 1;

    //TODO: Get the actual acceleration limit based on a timedetla.
    double max_velocity_accel_limit = (this->max_acceleration * timedelta) + this->vehicle.v();

    int car_ahead_id = this->get_vehicle_ahead(lane, other_vehicle_predictions);
    int car_behind_id = this->get_vehicle_ahead(lane, other_vehicle_predictions);
    FrenetPos car_frenet = this->trackMap.getFrenet(this->vehicle.position());

    double new_v;
    double new_a;

    // There is a car ahead.
    if (car_ahead_id) {
      Vehicle car_ahead = other_vehicle_predictions[car_ahead_id][0];
      FrenetPos car_ahead_frenet= this->trackMap.getFrenet(car_ahead.position());

      // There is also a car behind.
      if (car_behind_id) {
        Vehicle car_behind = other_vehicle_predictions[car_behind_id][0];
        new_v = car_behind.v(); //must travel at the speed of traffic, regardless of preferred buffer
      }
      // There is also no car behind.
      else {
        // TODO: This isn't right I think.. How is max velocity being calculated?
         double max_velocity_in_front =
             (car_ahead_frenet.s - car_frenet.s - this->preferred_buffer)
             + car_ahead.v() - 0.5 * (this->vehicle.a());
        new_v = min(max_velocity_in_front, max_velocity_accel_limit, this->target_speed);
      }
    // There is no car ahead.
    } else {
      new_v = min(max_velocity_accel_limit, this->target_speed);
    }

    // Get the new acceleration and postion based on the new velocity.
    new_a = new_v - this->vehicle.v() / timedelta; //Equation: (v_1 - v_0)/t = acceleration
    // Get new position after timedelta.
    car_frenet.s = car_frenet.s + (this->vehicle.v() * timedelta) + (0.5 * new_a * timedelta * timedelta);

    // Create the kinematic as a new Vehicle;
    Vehicle lane_kinematic = Vehicle(this->trackMap.getXY(car_frenet));
    lane_kinematic.v(new_v);
    lane_kinematic.a(new_a);

    return lane_kinematic;

  }

  vector<Vehicle> VehicleController::constant_speed_trajectory(map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Generate a constant speed trajectory.
    */

    double timedelta = 1.;

    // TODO: This should take something like number of increments and increment delta?
    Vehicle next_vehicle_in_trajectory = this->vehicle.clone();
    next_vehicle_in_trajectory.position(this->trackMap.position_at(this->vehicle.position(), timedelta));

    return  {this->vehicle.clone(), next_vehicle_in_trajectory};
  }

  vector<Vehicle> VehicleController::keep_lane_trajectory(map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Generate a keep lane trajectory.
    */

    double timedelta = 1;

    // First trajectory item
    vector<Vehicle> trajectory = {vehicle.clone()};

    // Second trajectory item
    trajectory.emplace_back(this->get_lane_kinematic(this->get_lane(), other_vehicle_predictions));

    return trajectory;
  }

  vector<Vehicle> VehicleController::prep_lane_change_trajectory(STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions) {

    int new_lane = this->get_lane() + LANE_DIRECTION[state];
    // First trajectory item
    vector<Vehicle> trajectory = {vehicle.clone()};
    // First get the next best trajectory for the current lane.
    Vehicle best_next_trajectory = get_lane_kinematic(this->get_lane(), other_vehicle_predictions);

    int car_behind_index = this->get_vehicle_behind(this->get_lane(), other_vehicle_predictions);

    if (! car_behind_index){
      vector<double> best_kinematics;
      vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
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

  vector<Vehicle> VehicleController::lane_change_trajectory(STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->get_lane() + lane_direction[state];
    vector<VehicleController> trajectory;
    VehicleController next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (auto &prediction : predictions) {
      next_lane_vehicle = prediction.second[0];
      if (next_lane_vehicle.s == this->s && next_lane_vehicle.get_lane() == new_lane) {
        //If lane change is not possible, return empty trajectory.
        return trajectory;
      }
    }
    trajectory.emplace_back(this->s, this->d, this->v, this->a, 0, this->state);
    vector<double> kinematics = get_kinematics(predictions, new_lane);
    trajectory.emplace_back(kinematics[0], this->d, kinematics[1], kinematics[2], 0, state);
    return trajectory;
  }

  void VehicleController::increment(double timedelta) {
    this->vehicle = this->trackMap->position_at(timedelta);
  }

  int VehicleController::get_vehicle_behind(int lane,  map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double max_s = -1;
    int closest_vehicle_found = 0; // All trajectories for cars don't start with 0, so 0 is not found.
    FrenetPos car = this->trackMap.getFrenet(this->vehicle.position());

    for (auto &trajectory : other_vehicle_predictions) {
      // Get the first trajectory item for this vehicle id.
      FrenetPos other_car = this->trackMap.getFrenet(trajectory.second[0].position());
      int other_car_lane = this->trackMap.getFrenetLane(other_car);

      if (other_car_lane == lane && other_car.s < car.s && other_car.s > max_s) {
        max_s = other_car.s;
        closest_vehicle_found = trajectory.first;
      }
    }
    return closest_vehicle_found;
  }

  int VehicleController::get_vehicle_ahead(int lane,  map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = RLIM_INFINITY;
    int closest_vehicle_found = 0; // All trajectories for cars don't start with 0, so 0 is not found.
    FrenetPos car = this->trackMap.getFrenet(this->vehicle.position());

    for (auto &trajectory : other_vehicle_predictions) {
      // FYI: for a map<first, second>, it->second[0] would be the first instance of the second type (so the first vehicle).
      // Get the first trajectory item for this vehicle id.
      FrenetPos other_car = this->trackMap.getFrenet(trajectory.second[0].position());
      int other_car_lane = this->trackMap.getFrenetLane(other_car);
      if (other_car_lane == lane && other_car.s > car.s && other_car.s < min_s) {
        min_s = other_car.s;
        closest_vehicle_found = trajectory.first;
      }
    }
    return closest_vehicle_found;
  }

  vector<Vehicle> VehicleController::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
    vector<VehicleController> predictions;
    for (int i = 0; i < horizon; i++) {
      double next_s = position_at(i);
      double next_v = 0;
      if (i < horizon - 1) {
        next_v = position_at(i + 1) - s;
      }
      predictions.emplace_back(this->get_lane(), next_s, next_v, 0);
    }
    return predictions;

  }

  void VehicleController::realize_next_state(vector<VehicleController> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    VehicleController next_state = trajectory[1];
    this->state = next_state.state;
    this->target_lane = next_state.get_lane();
    //this->s = next_state.s;
    this->target_speed = next_state.v;
    //this->a = next_state.a;
  }

  void VehicleController::configure(vector<double> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = (double) road_data[0];
    lanes_available = (int) road_data[1];
    target_s = (double) road_data[2];
    target_lane = (int) road_data[3];
    max_acceleration = (double) road_data[4];
    max_legal_speed = (double) road_data[5];
  }

  bool VehicleController::in_my_lane(VehicleController &other) {
    //const double lane_buffer = 0.5;
    int other_lane = other.get_lane();
    if (other_lane < 0) return false;
    int my_lane = this->get_lane();

    bool in_my_lane = (my_lane == other_lane);
    return in_my_lane;;
  }

  double VehicleController::distance_from_me(VehicleController &other, double time_delta) {
    double other_car_s = other.s;
    other_car_s += (time_delta * other.v); //if using time_delta, project car's s value out in time.
    return (other_car_s - this->s);
  }


  double VehicleController::accelerate(double factor) {
    double new_a = this->a + factor;
    if (abs(new_a) <= this->max_acceleration) {
      this->a = new_a;
    }
    return new_a;
  }

  double VehicleController::decelerate(double factor) {
    double new_a = this->a - factor;
    if (abs(new_a) <= this->max_acceleration) {
      this->a = new_a;
    }
    return new_a;
  }

  VehicleController VehicleController::clone() {
    return VehicleController(this->s, this->d, this->v, this->a, this->yaw, this->state);
  }


  void extend_car_path(VehicleController curr_car, VehicleController goal_car, utils::XYList &path, utils::Map &map) {

    auto prev_size = path.x_list.size();

    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    // Later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
    vector<utils::Position> way_pts;

    // Use a proxy for the current car position, this will be either where we really are, or the end of the active path.
    utils::Position curr_car_pt = curr_car.to;

    Position prev_car_pt;
    // If the previous points are almost empty, use the car as a starting reference
    // and infer two points based on the car's current position and heading.
    if (prev_size < 2) {
      // Use the two points that make the path tangent to the car.
      prev_car_pt = Position(curr_car_pt.x - cos(curr_car_pt.yaw), curr_car_pt.y - sin(curr_car_pt.yaw));

      way_pts.push_back(prev_car_pt);
      way_pts.push_back(curr_car_pt);

    } else {
      // Otherwise, use the previous path's endpoint as a starting reference.

      // Redefine the reference state as the previous path endpoint.
      prev_car_pt = Position(previous_path_x[prev_size - 1], previous_path_y[prev_size - 1]);
      curr_car_pt = Position(previous_path_x[prev_size - 2], previous_path_y[prev_size - 2]);
      curr_car_pt.yaw = atan2(curr_car_pt.y - prev_car_pt.y, curr_car_pt.x - prev_car_pt.x);

      way_pts.push_back(prev_car_pt);
      way_pts.push_back(curr_car_pt);

    }

    // In FrenetPos, add evenly 60m spaced points ahead of the starting reference. (30 gave errors sometimes)
    way_pts.push_back(getXY(current_s + 60, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y));
    way_pts.push_back(getXY(current_s + 120, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y));
    way_pts.push_back(getXY(current_s + 180, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y));


    // Shift to the car's coordinates to make the math easier later.
    for (auto &way_pt : way_pts) {

      // Shift the car reference angle to 0 degrees.
      way_pt = way_pt.convert_to_frame(curr_car_pt);
    }

    Spline spline = Spline(way_pts);

    // Define the actual x,y points we'll be using for the planner.
    // Start by filling the next_x_vals with what's left over from previous path.
    for (int i = 0; i < previous_path_x.size(); i++) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);

    }

    double target_x = 30.;
    Position pt = spline.interpolate(target_x);
    double target_dist = sqrt(pt.x * pt.x + pt.y * pt.y);

    double x_add_on = 0;

    // Fill up the rest of our path planner after filling it with previous points,
    // here we will always output 50 points.
    // TODO: (I reduced this to 5 as the acceleration kicked in too hard at the end of the first
    // batch of paths..  we should account for acceleration!
    const int INCREMENTS = 50;
    double acceleration_inc = acceleration / INCREMENTS;

    for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
      velocity += acceleration_inc;

      double N = (target_dist / (.02 * velocity));

      double x_point = ((target_x / N) + x_add_on);
      Position next_pt = spline.interpolate(x_point)
          .convert_from_frame(car_pt);

      x_add_on = x_point;

      next_x_vals.push_back(next_pt.x);
      next_y_vals.push_back(next_pt.y);
    }
  }


}


