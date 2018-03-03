#include "vehicle.h"
#include <algorithm>
#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include "fsm.h"
#include "utils.h"

using namespace utils;
using namespace std;
using namespace fsm;
using namespace std::chrono;

namespace vehicle {

/**
 * Initializes Vehicle
 */

  Vehicle::Vehicle() {
    this->_v =0;
    this->_a =0;
    this->_yaw_delta =0;
    this->_time = system_clock::now();
  };

  Vehicle::Vehicle(int id, Position pos) {
    this->_id = id;
    this->_position = pos;
  }

  int Vehicle::id() {
    return this->_id;
  }

  void Vehicle::id(int id) {
    this->_id = id;
  }

  time_point<system_clock> Vehicle::time() {
    return this->_time;
  };

  void Vehicle::time(time_point<system_clock> time) {
    this->_time = time;
  };

  void Vehicle::addSeconds(double seconds) {
    this->_time += chrono::microseconds((int) (seconds * 1000000));
  };

  double Vehicle::secondsDiff(time_point<system_clock> other_time) {
    return (other_time - this->_time).count() / 1000000.;
  };


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
    this->_v = v;
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

    Vehicle new_vehicle = Vehicle(this->id(), this->position().clone());
    new_vehicle.v(this->v());
    new_vehicle.a(this->a());
    new_vehicle.yaw_delta(this->yaw_delta());
    new_vehicle.time(this->time());
    return new_vehicle;
  }

/**
 * Initializes VehicleController
 */

  VehicleController::VehicleController(Vehicle vehicle, VehicleFSM *fsm, Map *trackMap) {
    this->vehicle = vehicle;
    this->fsm = fsm;
    this->trackMap = trackMap;
    this->last_vehicle = vehicle.clone();
  }

  int VehicleController::get_lane() {
    return this->trackMap->getXYLane(this->vehicle.position());
  }

  void VehicleController::update(double x, double y, double yaw, double speed_mph, int prev_size) {
    this->last_update_time_delta = duration_cast<std::chrono::milliseconds>(system_clock::now() - this->last_update_time).count();
    this->last_update_time = system_clock::now();
    //cout << time_diff<< "ms \n";

    this->last_vehicle = vehicle.clone();

    this->vehicle.x(x);
    this->vehicle.y(y);
    this->vehicle.yaw(yaw);
    this->vehicle.v(utils::from_mph(speed_mph));

    this->trim_prev_trajectory(prev_size);

  };

  std::pair<fsm::STATE, vector<Vehicle>> VehicleController::choose_next_state(map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<fsm::STATE> states = this->fsm->successor_states(this->get_lane());
    double cost;
    vector<double> costs;
    vector<fsm::STATE> final_states;
    vector<vector<Vehicle>> final_trajectories;

    for (fsm::STATE &state : states) {
      vector<Vehicle> candidate_trajectory = this->generate_trajectory(state, other_vehicle_predictions);
      if (!candidate_trajectory.empty()) {
        cost = this->calculate_cost(candidate_trajectory, other_vehicle_predictions);
        costs.push_back(cost);
        final_trajectories.push_back(candidate_trajectory);
        final_states.push_back(state);
      }
    }

    auto best_cost = min_element(begin(costs), end(costs));
    auto best_idx = (int) distance(begin(costs), best_cost);

    return {final_states[best_idx], final_trajectories[best_idx]};
  }

  Vehicle VehicleController::predict_next(Vehicle &car, double timedelta) {
    FrenetPos car_frenet = this->trackMap->getFrenet(car.position());
    car_frenet.s = car_frenet.s + (car.v() * timedelta) + (0.5 * car.a() * timedelta * timedelta);
    Vehicle next_car = car.clone();
    next_car.v((car.a() * timedelta) + car.v());
    next_car.position(this->trackMap->getXY(car_frenet));
    next_car.addSeconds(timedelta);
    return next_car;
  }

  void VehicleController::trim_prev_trajectory(int remaining_size) {
    int items_to_pop = (int) this->trajectory.size() - remaining_size;
    for(int i=0; i< items_to_pop; i++) {
      this->trajectory.pop_front();
    }
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

  Vehicle VehicleController::get_lane_kinematic(int lane, double timedelta, double target_velocity, map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */

    //TODO: Get the actual acceleration limit based on a timedetla.
    double max_velocity_accel_limit = (this->max_acceleration * timedelta) + this->vehicle.v();

    int car_ahead_id = this->get_vehicle_ahead(lane, other_vehicle_predictions);
    int car_behind_id = this->get_vehicle_ahead(lane, other_vehicle_predictions);
    FrenetPos car_frenet = this->trackMap->getFrenet(this->vehicle.position());

    double new_v;
    double new_a;

    // There is a car ahead.
    if (car_ahead_id) {
      Vehicle car_ahead = other_vehicle_predictions[car_ahead_id][0];
      FrenetPos car_ahead_frenet= this->trackMap->getFrenet(car_ahead.position());

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
        new_v = min(max_velocity_in_front, min(max_velocity_accel_limit, target_velocity));
      }
    // There is no car ahead.
    } else {
      new_v = min(max_velocity_accel_limit, target_velocity);
    }

    // Get the new acceleration and postion based on the new velocity.
    new_a = new_v - this->vehicle.v() / timedelta; //Equation: (v_1 - v_0)/t = acceleration
    // Get new position after timedelta.
    car_frenet.s = car_frenet.s + (this->vehicle.v() * timedelta) + (0.5 * new_a * timedelta * timedelta);

    // Create the kinematic as a new Vehicle;
    Vehicle lane_kinematic = Vehicle(this->vehicle.id(), this->trackMap->getXY(car_frenet));
    lane_kinematic.v(new_v);
    lane_kinematic.a(new_a);
    lane_kinematic.addSeconds(timedelta);

    return lane_kinematic;

  }

  /** TRAJECTORY GENERATION **/

  vector<Vehicle> VehicleController::constant_speed_trajectory(map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Generate a constant speed trajectory.
    */

    double timedelta = 1.;
    double steps = 3;
    vector<Vehicle> trajectory{this->vehicle.clone()};

    for (int i=0; i<steps; i++) {
      Vehicle next_vehicle = trajectory[0].clone();
      next_vehicle.position(this->trackMap->position_at(next_vehicle.position(), timedelta));
      next_vehicle.addSeconds(timedelta);
      trajectory.push_back(next_vehicle);
    }

    return trajectory;
  }

  vector<Vehicle> VehicleController::keep_lane_trajectory(map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Generate a keep lane trajectory.
    */

    double timedelta = 1.;
    double steps = 3;
    vector<Vehicle> trajectory{this->vehicle.clone()};

    for (int i=0; i<steps; i++) {
      trajectory.push_back(this->get_lane_kinematic(this->get_lane(), timedelta, INFINITY, other_vehicle_predictions));
      timedelta += 1.;
    }

    return trajectory;
  }

  vector<Vehicle> VehicleController::prep_lane_change_trajectory(STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions) {

    double timedelta = 1.;
    int new_lane = this->get_lane() + LANE_DIRECTION.at(state);
    // First trajectory item
    vector<Vehicle> trajectory = {vehicle.clone()};
    // First get the next best trajectory for the current lane.
    Vehicle best_curr_lane_kinematic = get_lane_kinematic(this->get_lane(), timedelta, INFINITY, other_vehicle_predictions);

    int car_behind_index = this->get_vehicle_behind(this->get_lane(), other_vehicle_predictions);

    if (! car_behind_index){
      Vehicle best_new_lane_kinematic = get_lane_kinematic(new_lane, timedelta, INFINITY, other_vehicle_predictions);
      //Choose kinematics with lowest velocity. // TODO WHY?
      if (best_new_lane_kinematic.v() < best_curr_lane_kinematic.v()) {
        trajectory.emplace_back(best_new_lane_kinematic);
      } else {
        trajectory.emplace_back(best_curr_lane_kinematic);
      }
    }
    return trajectory;
  }

  vector<Vehicle> VehicleController::lane_change_trajectory(STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Generate a lane change trajectory.
    */
    double timedelta = 1;
    int new_lane = this->get_lane() + LANE_DIRECTION.at(state);
    // First trajectory item
    vector<Vehicle> trajectory = {};

    //Check if a lane change is possible (check if another vehicle occupies that spot).
    if (! this->lane_opening_exists(new_lane, other_vehicle_predictions)) {
      return trajectory;
    }
    trajectory.emplace_back(this->vehicle.clone());
    Vehicle kinematic = get_lane_kinematic(new_lane, timedelta, INFINITY, other_vehicle_predictions);
    trajectory.emplace_back(kinematic);
    return trajectory;
  }

  int VehicleController::get_vehicle_behind(int lane,  map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double max_s = -1;
    int closest_vehicle_found = 0; // All trajectories for cars don't start with 0, so 0 is not found.
    FrenetPos car = this->trackMap->getFrenet(this->vehicle.position());

    for (auto &trajectory : other_vehicle_predictions) {
      // Get the first trajectory item for this vehicle id.
      FrenetPos other_car = this->trackMap->getFrenet(trajectory.second[0].position());
      int other_car_lane = this->trackMap->getFrenetLane(other_car);

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
    FrenetPos car = this->trackMap->getFrenet(this->vehicle.position());

    for (auto &trajectory : other_vehicle_predictions) {
      // FYI: for a map<first, second>, it->second[0] would be the first instance of the second type (so the first vehicle).
      // Get the first trajectory item for this vehicle id.
      FrenetPos other_car = this->trackMap->getFrenet(trajectory.second[0].position());
      int other_car_lane = this->trackMap->getFrenetLane(other_car);
      if (other_car_lane == lane && other_car.s > car.s && other_car.s < min_s) {
        min_s = other_car.s;
        closest_vehicle_found = trajectory.first;
      }
    }
    return closest_vehicle_found;
  }

  bool VehicleController::lane_opening_exists(int lane,  map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    FrenetPos car = this->trackMap->getFrenet(this->vehicle.position());

    for (auto &trajectory : other_vehicle_predictions) {
      // FYI: for a map<first, second>, it->second[0] would be the first instance of the second type (so the first vehicle).
      // Get the first trajectory item for this vehicle id.
      FrenetPos other_car = this->trackMap->getFrenet(trajectory.second[0].position());
      int other_car_lane = this->trackMap->getFrenetLane(other_car);

      // We're only concerned with the cars in a specific lane.
      if (other_car_lane == lane) {
        // If the car is behind, but there isn't room then return false.
        if (other_car.s <= car.s && (other_car.s + this->preferred_buffer) >= car.s) {
          return false;
        // OR if there is a car in front but there isn't room, return false.
        } else if (other_car.s >= car.s && (other_car.s - this->preferred_buffer) <= car.s) {
          return false;
        }
      }
    }
    // If no car matches, then there is room.
    return true;
  }

  vector<Vehicle> VehicleController::generate_predictions(int n_steps, Vehicle other_car) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */

    double timedelta = 1.;

    vector<Vehicle> predictions;

    for (int i = 0; i < n_steps; i++) {
      Vehicle next = this->predict_next(this->vehicle, timedelta);
      predictions.emplace_back(next);
    }
    return predictions;

  }

  void VehicleController::extend_trajectory(vector<Vehicle> &path) {

    //auto prev_size = this->trajectory.size();

    vector<double> time_way_pts{};
    vector<double> s_way_pts{};
    vector<double> d_way_pts{};
    vector<double> v_way_pts{};

    // Create separate splines for the x and y coordinates.
    for(int i=0; i < path.size(); i++){
      FrenetPos frenet = this->trackMap->getFrenet(path[i].position());
      time_way_pts.emplace_back((double) (path[0].time() - path[i].time()).count());
      s_way_pts.emplace_back(frenet.s);
      d_way_pts.emplace_back(frenet.d);
      v_way_pts.emplace_back(path[i].v());
    }

    tk::spline s_spline{};
    tk::spline d_spline{};
    tk::spline v_spline{};

    s_spline.set_points(time_way_pts, s_way_pts);
    d_spline.set_points(time_way_pts, d_way_pts);
    v_spline.set_points(time_way_pts, v_way_pts);

    const double total_time = 1.;
    const double time_step_size = 0.02; // in seconds.
    double time_left = total_time - (this->trajectory.size() * 0.02);

    if (time_left > time_step_size) {

      auto remaining_increments = (int) round(time_left / time_step_size);

      for (int inc = 1; inc < remaining_increments; inc++) {
        auto next_frenet = FrenetPos{
            s_spline(inc * time_step_size),
            d_spline(inc * time_step_size)
        };
        auto next_trajectory_item = Vehicle{0, this->trackMap->getXY(next_frenet)};
        next_trajectory_item.v(v_spline(inc * time_step_size));
        this->trajectory.push_back(next_trajectory_item);
      }
    }
  }


  double VehicleController::calculate_cost(vector<Vehicle> &candidate_trajectory, std::map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    double cost = 0.0;

//    //Add additional cost functions here.
//    vector<function<double(const VehicleController &, const vector<VehicleController> &,
//                           const map<int, vector<VehicleController>> &, map<string, double> &)>> cf_list = {
//        goal_distance_cost, inefficiency_cost};
//    vector<double> weight_list = {REACH_GOAL, EFFICIENCY};
//
//    for (int i = 0; i < cf_list.size(); i++) {
//      double new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, predictions, trajectory_data);
//      cost += new_cost;
//    }

    return cost;
  }
}


