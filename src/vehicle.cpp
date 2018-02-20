#include <algorithm>
#include <iostream>
#include <utility>
#include "vehicle.h"
#include "fsm.h"
#include "utils.h"

using namespace utils;

namespace vehicle {

/**
 * Initializes Vehicle
 */

  Vehicle::Vehicle()=default;

  Vehicle::Vehicle(Point location) : location(location) {}

  double Vehicle::v() {
    return sqrt(dx * dx + dy * dy);
  }

  double Vehicle::x() {
    return location.x;
  }

  double Vehicle::y() {
    return location.y;
  }

  double Vehicle::yaw() {
    return location.yaw;
  }

  VehicleController::VehicleController(Vehicle &vehicle, fsm::VehicleFSM fsm, Map &map) {
    this->vehicle = vehicle;
    this->fsm = fsm;
    this->map = map;
  }

  vector<VehicleController> VehicleController::choose_next_state(map<int, vector<Vehicle>> other_vehicle_predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> states = this->fsm->successor_states();
    double cost;
    vector<double> costs;
    vector<string> final_states;
    vector<vector<VehicleController>> final_trajectories;

    for (auto &state : states) {
      vector<VehicleController> trajectory = generate_trajectory(state, other_vehicle_predictions);
      if (!trajectory.empty()) {
        cost = calculate_cost(*this, other_vehicle_predictions, trajectory);
        costs.push_back(cost);
        final_trajectories.push_back(trajectory);
      }
    }

    auto best_cost = min_element(begin(costs), end(costs));
    auto best_idx = (int) distance(begin(costs), best_cost);
    return final_trajectories[best_idx];
  }

  vector<Vehicle> VehicleController::generate_trajectory(fsm::STATE state, map<int, vector<Vehicle> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state == fsm::STATE::CS) {
      trajectory = constant_speed_trajectory();
    } else if (state == fsm::STATE::KL) {
      trajectory = keep_lane_trajectory(predictions);
    } else if (state == fsm::STATE::LCL || state == fsm::STATE::LCR) {
      trajectory = lane_change_trajectory(state, predictions);
    } else if (state == fsm::STATE::PLCL || state == fsm::STATE::PLCL) {
      trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
  }

  vector<double> VehicleController::get_kinematics(map<int, vector<VehicleController>> predictions, int lane) {
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    double max_velocity_accel_limit = this->max_acceleration + this->v;
    double new_position;
    double new_velocity;
    double new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

      if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
        new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
      } else {
        auto max_velocity_in_front = (double) ((vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v -
                                               0.5 * (this->a));
        new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
      }
    } else {
      new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }

    new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / (double) 2.0;
    return {new_position, new_velocity, new_accel};

  }

  vector<VehicleController> VehicleController::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    double next_pos = position_at(1);
    vector<VehicleController> trajectory = {VehicleController(this->s, this->d, this->v, this->a, 0, this->state),
                                            VehicleController(next_pos, this->d, this->v, 0, 0, this->state)};
    return trajectory;
  }

  vector<VehicleController> VehicleController::keep_lane_trajectory(map<int, vector<VehicleController>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    vector<VehicleController> trajectory = {VehicleController(this->s, this->d, this->v, this->a, 0, state)};
    vector<double> kinematics = get_kinematics(predictions, (int) round(this->get_lane()));
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    trajectory.emplace_back(new_s, this->d, new_v, new_a, 0, "KL");
    return trajectory;
  }

  vector<VehicleController>
  VehicleController::prep_lane_change_trajectory(string state, map<int, vector<VehicleController>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    double new_s;
    double new_v;
    double new_a;
    VehicleController vehicle_behind;
    int new_lane = this->get_lane() + lane_direction[state];
    vector<VehicleController> trajectory = {VehicleController(this->s, this->d, this->v, this->a, 0, this->state)};
    vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->get_lane());

    if (get_vehicle_behind(predictions, this->get_lane(), vehicle_behind)) {
      //Keep speed of current lane so as not to collide with car behind.
      new_s = curr_lane_new_kinematics[0];
      new_v = curr_lane_new_kinematics[1];
      new_a = curr_lane_new_kinematics[2];

    } else {
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

  vector<VehicleController>
  VehicleController::lane_change_trajectory(string state, map<int, vector<VehicleController>> predictions) {
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

  void VehicleController::increment(int dt = 1) {
    this->s = position_at(dt);
  }

  double VehicleController::position_at(int t) {
    return this->s + this->v * t + this->a * t * t / (double) 2.0;
  }

  bool VehicleController::get_vehicle_behind(VehicleController &rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double max_s = -1;
    bool found_vehicle = false;
    VehicleController temp_vehicle;
    for (auto &it : other_cars) {
      // FYI: for a map<first, second>, it->second[0] would be the first instance of the second type (so the first vehicle).
      VehicleController other_car = it.second[0];

      if (temp_vehicle.get_lane() == this->get_lane() && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
        max_s = temp_vehicle.s;
        rVehicle = temp_vehicle;
        found_vehicle = true;
      }
    }
    return found_vehicle;
  }

  bool VehicleController::get_vehicle_ahead(map<int, vector<VehicleController>> predictions, int lane,
                                            VehicleController &rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = this->target_s;
    bool found_vehicle = false;
    VehicleController temp_vehicle;
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

  vector<VehicleController> VehicleController::generate_predictions(int horizon) {
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

  int VehicleController::get_lane() {
    // Lanes are 4m wide, so first divide and then subtract 1/2 a lane to center.
    // assumes 0 is the left most lane.
    return VehicleController::d_to_lane(this->d);
  }

  double VehicleController::lane_to_d(double lane) {
    // Lanes are 4m wide, so first multiply by 4 and then add 2 to center it.
    // assumes 0 is the left most lane.
    return lane * 4 + 2;
  }

  int VehicleController::d_to_lane(double d) {
    // Lanes are 4m wide, so first multiply by 4 and then add 2 to center it.
    // assumes 0 is the left most lane.
    return (int) round(d / 4 - 0.5);
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
    vector<utils::Point> way_pts;

    // Use a proxy for the current car position, this will be either where we really are, or the end of the active path.
    utils::Point curr_car_pt = curr_car.to;

    Point prev_car_pt;
    // If the previous points are almost empty, use the car as a starting reference
    // and infer two points based on the car's current position and heading.
    if (prev_size < 2) {
      // Use the two points that make the path tangent to the car.
      prev_car_pt = Point(curr_car_pt.x - cos(curr_car_pt.yaw), curr_car_pt.y - sin(curr_car_pt.yaw));

      way_pts.push_back(prev_car_pt);
      way_pts.push_back(curr_car_pt);

    } else {
      // Otherwise, use the previous path's endpoint as a starting reference.

      // Redefine the reference state as the previous path endpoint.
      prev_car_pt = Point(previous_path_x[prev_size - 1], previous_path_y[prev_size - 1]);
      curr_car_pt = Point(previous_path_x[prev_size - 2], previous_path_y[prev_size - 2]);
      curr_car_pt.yaw = atan2(curr_car_pt.y - prev_car_pt.y, curr_car_pt.x - prev_car_pt.x);

      way_pts.push_back(prev_car_pt);
      way_pts.push_back(curr_car_pt);

    }

    // In Frenet, add evenly 60m spaced points ahead of the starting reference. (30 gave errors sometimes)
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
    Point pt = spline.interpolate(target_x);
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
      Point next_pt = spline.interpolate(x_point)
          .convert_from_frame(car_pt);

      x_add_on = x_point;

      next_x_vals.push_back(next_pt.x);
      next_y_vals.push_back(next_pt.y);
    }
  }


}


