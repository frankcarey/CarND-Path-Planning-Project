#include "vehicle.h"
#include <algorithm>
#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include "fsm.h"
//####include "utils.h"

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
    this->_yaw = 0;
    this->_yaw_delta = 0;
    this->_time = system_clock::now();
  };

  //Note: The second part is the constuctor call in C++11.
  Vehicle::Vehicle(int id, FrenetPos pos) : Vehicle() {
    this->_id = id;
    this->_fpos = pos;
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


  double Vehicle::s() {
    return this->_fpos.s;
  }

  void Vehicle::s(double s) {
    this->_fpos.s = s;
  }

  double Vehicle::d() {
    return this->_fpos.d;
  }

  void Vehicle::d(double d) {
    this->_fpos.d = d;
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

  double Vehicle::yaw() {
    return this->_yaw;
  }

  void Vehicle::yaw(double yaw) {
    this->_yaw = yaw;
  }

  double Vehicle::yaw_delta() {
    // TODO: Get the actual value.
    return this->_yaw_delta;
  }

  void Vehicle::yaw_delta(double yaw_delta) {
    this->_yaw_delta = yaw_delta;
  }

  FrenetPos Vehicle::position() {
    // TODO: Get the actual value.
    return this->_fpos;
  }

  void Vehicle::position(FrenetPos pos) {
    this->_fpos = pos;
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

    //this->last_vehicle = vehicle.clone();
  }

  int VehicleController::get_lane() {
    return this->trackMap->getFrenetLane(this->vehicle.position());
  }

  void VehicleController::update(double s, double d, double yaw, double speed_mph, int prev_path_size) {
    this->last_update_time = system_clock::now();
    this->vehicle.s(s);
    this->vehicle.d(d);
    this->vehicle.yaw(yaw);
    this->vehicle.v(utils::from_mph(speed_mph));
    this->prev_path_size = prev_path_size;
  };

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

  Vehicle VehicleController::get_lane_kinematic(Vehicle vehicle, int lane, double timedelta, double target_velocity, map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */

    Vehicle lane_kinematic{1, {}};
    return lane_kinematic;
    //TODO: Get the actual acceleration limit based on a timedelta.
//    double max_velocity_accel_limit = (this->max_acceleration * timedelta) + vehicle.v();
//
//    int car_ahead_id = this->get_vehicle_ahead(lane, other_vehicle_predictions);
//    int car_behind_id = this->get_vehicle_ahead(lane, other_vehicle_predictions);
//    FrenetPos car_frenet = this->trackMap->getFrenet(vehicle.position());
//
//    double new_v;
//    double new_a;
//
//    // There is a car ahead.
//    if (car_ahead_id) {
//      Vehicle car_ahead = other_vehicle_predictions[car_ahead_id][0];
//      FrenetPos car_ahead_frenet= this->trackMap->getFrenet(car_ahead.position());
//
//      // There is also a car behind.
////      if (car_behind_id) {
////        Vehicle car_behind = other_vehicle_predictions[car_behind_id][0];
////        new_v = car_behind.v(); //must travel at the speed of traffic, regardless of preferred buffer
////      }
////      // There is also no car behind.
////      else {
//        // TODO: This isn't right I think.. How is max velocity being calculated?
//         double max_velocity_in_front =
//             (car_ahead_frenet.s - car_frenet.s - this->preferred_buffer) + car_ahead.v();
//        new_v = min(max_velocity_in_front, min(max_velocity_accel_limit, target_velocity));
//      //}
//    // There is no car ahead.
//    } else {
//      new_v = min(max_velocity_accel_limit, target_velocity);
//    }
//
//    // Get the new acceleration and postion based on the new velocity.
//    new_a = (new_v - vehicle.v()) / timedelta; //Equation: (v_1 - v_0)/t = acceleration
//    // Get new position after timedelta.
//    //car_frenet.s = car_frenet.s + (this->vehicle.v() * timedelta) + (0.5 * new_a * timedelta * timedelta);
//    double new_d = lane * 4 + 2;
//    car_frenet.s = car_frenet.s + (new_v * timedelta);
//    car_frenet.d = car_frenet.d + (car_frenet.d - new_d) * timedelta;
//
//    // Create the kinematic as a new Vehicle;
//    Position new_pos = this->trackMap->getXY(car_frenet);
//    int vid = vehicle.id();
//    Vehicle lane_kinematic{vid, new_pos};
//
//    lane_kinematic.v(new_v);
//    lane_kinematic.a(new_a);
//    lane_kinematic.time(vehicle.time());
//
//    lane_kinematic.addSeconds(timedelta);
//
//    //cout << "\nTEST : " << lane_kinematic.id();
//
//    return lane_kinematic;

  }

  /** TRAJECTORY GENERATION **/

  vector<Vehicle> VehicleController::constant_speed_trajectory(map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Generate a constant speed trajectory.
    */

    double timedelta = 1.;
    double steps = 3;
    vector<Vehicle> trajectory{};
    return trajectory;
//    vector<Vehicle> trajectory{this->vehicle.clone()};
//
//    for (int i=0; i<steps; i++) {
//      Vehicle next_vehicle = trajectory[0].clone();
//      next_vehicle.position(this->trackMap->position_at(next_vehicle.position(), timedelta));
//      next_vehicle.addSeconds(timedelta);
//      trajectory.push_back(next_vehicle);
//    }
//
//    return trajectory;
  }

  vector<Vehicle> VehicleController::keep_lane_trajectory(map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Generate a keep lane trajectory.
    */

    cout << "\nKeep lane\n";

    double timedelta = .02;
    double steps = 50;
    //vector<Vehicle> trajectory{this->vehicle.clone()};
    vector<Vehicle> trajectory{};

    for (int i=1; i<=steps; i++) {
      trajectory.push_back(this->get_lane_kinematic(this->vehicle, this->get_lane(), timedelta*i, this->trackMap->speed_limit(), other_vehicle_predictions));
    }

    return trajectory;
  }

  vector<Vehicle> VehicleController::prep_lane_change_trajectory(STATE state, map<int, vector<Vehicle>> &other_vehicle_predictions) {

    double timedelta = 1.;
    int new_lane = this->get_lane() + LANE_DIRECTION.at(state);
    // First trajectory item
    vector<Vehicle> trajectory = {vehicle.clone()};
    // First get the next best trajectory for the current lane.
    Vehicle best_curr_lane_kinematic = get_lane_kinematic(this->vehicle, this->get_lane(), timedelta, INFINITY, other_vehicle_predictions);

    int car_behind_index = this->get_vehicle_behind(this->get_lane(), other_vehicle_predictions);

    if (! car_behind_index){
      Vehicle best_new_lane_kinematic = get_lane_kinematic(this->vehicle, new_lane, timedelta, INFINITY, other_vehicle_predictions);
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
    double timedelta = .02;
    double steps = 50;

    int new_lane = this->get_lane() + LANE_DIRECTION.at(state);
    // First trajectory item
    vector<Vehicle> trajectory = {};

    //Check if a lane change is possible (check if another vehicle occupies that spot).
    if (! this->lane_opening_exists(new_lane, other_vehicle_predictions)) {
      return trajectory;
    }
    for (int i=1; i<=steps; i++) {
      trajectory.push_back(this->get_lane_kinematic(this->vehicle, new_lane, timedelta*i, this->trackMap->speed_limit(), other_vehicle_predictions));
    }
    return trajectory;
  }

  int VehicleController::get_vehicle_behind(int lane,  map<int, vector<Vehicle>> &other_vehicle_predictions) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double max_s = -1;
    int closest_vehicle_found = 0; // All trajectories for cars don't start with 0, so 0 is not found.
    FrenetPos car = this->vehicle.position();

    for (auto &trajectory : other_vehicle_predictions) {
      // Get the first trajectory item for this vehicle id.
      FrenetPos other_car = trajectory.second[0].position();
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
    FrenetPos car = this->vehicle.position();

    for (auto &trajectory : other_vehicle_predictions) {
      // FYI: for a map<first, second>, it->second[0] would be the first instance of the second type (so the first vehicle).
      // Get the first trajectory item for this vehicle id.
      FrenetPos other_car = trajectory.second[0].position();
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
    FrenetPos car = this->vehicle.position();

    for (auto &trajectory : other_vehicle_predictions) {
      // FYI: for a map<first, second>, it->second[0] would be the first instance of the second type (so the first vehicle).
      // Get the first trajectory item for this vehicle id.
      FrenetPos other_car = trajectory.second[0].position();
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
}


