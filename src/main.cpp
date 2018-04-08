#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "vehicle.h"
#include "utils.h"
#include "path_planner.h"


using namespace std;
using namespace std::chrono;
using namespace vehicle;
using namespace fsm;

// for convenience
using json = nlohmann::json;


int main() {
  uWS::Hub h;

  // Waypoint map to read from
  string map_file = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  const double MAX_LEGAL_MPH = 50; // in MPH! TODO this is actually 50.
  const int NUMBER_OF_LANES = 3;
  const double MAX_ACCELERATION = 1; // in meters per second squared



  utils::Map trackMap =  utils::Map(map_file, max_s, MAX_LEGAL_MPH, NUMBER_OF_LANES);


  Vehicle car{};
  VehicleFSM fsm{};

  //car.configure(road_data);
  VehicleController carCtl = VehicleController(car, &fsm, &trackMap);
  // TODO: Initialize this initial position in a better way.
  //carCtl.vehicle = Vehicle(0, Position(909.47, 1128.67));
  carCtl.max_acceleration = MAX_ACCELERATION;

  bool initialized = false;

  int counter = 0;

  h.onMessage([&carCtl, &initialized, &counter](uWS::WebSocket<uWS::SERVER> ws, const char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length > 2 && data[0] == '4' && data[1] == '2') {
      auto str = utils::hasData(data);

      if (!str.empty()) {
        auto j = json::parse(str);

        string event = j[0].get<string>();

        if (event == "telemetry") {

          counter += 1;
          cout << "COUNTER :" << counter << "\n";

          json msgJson;

          //cout << j.dump() << " : JSON\n";

          double x = j[1]["x"];
          double y = j[1]["y"];
          double yaw_deg = j[1]["yaw"]; // The simulator can send deg > 360!
          double yaw_radians = utils::normalize_rad(utils::deg2rad(yaw_deg));
          double speed_mph = j[1]["speed"];
          double s = j[1]["s"];
          double d = j[1]["d"];

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          auto planner = PathPlanner();

          // j[1] is the data JSON object
          // Main car's localization Data
          cout << "simulator| x: " << x << " y:" << y << " s: " << s << " d: " << d << " deg: " << yaw_deg << " rad: "
               << yaw_radians << "\n";

          carCtl.update(x, y, yaw_radians, speed_mph, j[1]["previous_path_x"].size());
          // TODO: uncommment this and fix the interpolated waypoints. they seem to break near
          // new waypoints, probably because the map is wrong?
          //carCtl.trackMap->update_local_waypoints(carCtl.vehicle.position(), 2, 2);

          cout << "post-update| x: " << carCtl.vehicle.x() << " y:" << carCtl.vehicle.y() << " yaw:"
               << carCtl.vehicle.yaw() << "\n";
          FrenetPos carF = carCtl.trackMap->getFrenet(carCtl.vehicle.position());
          cout << "post-update-frenet| s: " << carF.s << " d:" << carF.d << "\n";
          Position carPos = carCtl.trackMap->getXY(carF);
          cout << "post-update| x: " << carPos.x << " y:" << carPos.y << " yaw:" << carPos.yaw << "\n";

          // Just return so we get a previous state.
          if (!initialized) {
            carCtl.last_path_vehicle = carCtl.vehicle.clone();
            initialized = true;
          } else {

            // TODO: These s and d values are actually shit! There seem to be a lot of complaints on the forums
            // See https://discussions.udacity.com/t/erratic-end-path-s/348927
            // Previous path's end s and d values
//            double end_path_s = j[1]["end_path_s"];
//            double end_path_d = j[1]["end_path_d"];

//            auto future_car = carCtl.vehicle.clone();
//            auto future_carCtl = VehicleController(future_car, carCtl.fsm, carCtl.trackMap);
//
//            if (end_path_s) {
//              cout << "end_path= s:" << end_path_s << " d: " << end_path_d << "\n";
//              Position end_path = carCtl.trackMap->getXY(FrenetPos(end_path_s, end_path_d));
//              cout << "end_path= x:" << end_path.x << " y: " << end_path.y << " yaw: " << end_path.yaw << "\n";
//
//
//              future_carCtl.vehicle.position(future_carCtl.trackMap->getXY(FrenetPos(end_path_s, end_path_d)));
//            }
//
//            auto planner = PathPlanner();

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            //["sensor_fusion"] A 2d vector of cars and then that car's [
            // * [0] car's unique ID
            // * [1] car's x position in map coordinates
            // * [2] car's y position in map coordinates
            // * [3] car's x velocity in m/s
            // * [4] car's y velocity in m/s
            // * [5] car's s position in frenet coordinates. (DO NOT USE)
            // * [6] car's d position in frenet coordinates. (DO NOT USE)
            auto sensor_fusion = j[1]["sensor_fusion"];
            vector<Vehicle> other_vehicles;
            for (auto &vehicle_data : sensor_fusion) {
              Vehicle new_vehicle{vehicle_data[0], Position{vehicle_data[1], vehicle_data[2]}};
              new_vehicle.v(utils::distance(vehicle_data[3], vehicle_data[4]));

              other_vehicles.emplace_back(new_vehicle);
            }

            // Generate the predictions for the other cars.
            map<int, vector<Vehicle>> other_vehicle_predictions;
            for (Vehicle &other_vehicle: other_vehicles) {
              other_vehicle_predictions[other_vehicle.id()] = carCtl.generate_predictions(3, other_vehicle);
            }

            FrenetPos lastF;
            Position lastPosXY;


            int prev_path_size = j[1]["previous_path_x"].size();
            cout << "prev_path_size: " << prev_path_size << "\n";
            int generate_path_size = 200 - prev_path_size;
            cout << "generate_path_size: " << generate_path_size << "\n";
            if (prev_path_size <= 0) {
              lastPosXY = carCtl.vehicle.position();
            } else {
              lastPosXY = Position{
                  j[1]["previous_path_x"][prev_path_size - 1],
                  j[1]["previous_path_y"][prev_path_size - 1],
                  // TODO using this yaw may not be accurate enough!
                  carCtl.vehicle.yaw()
              };
            }
            cout << "last_x: " << lastPosXY.x << " last y: " << lastPosXY.y << " last yaw: " << lastPosXY.yaw << "\n";
            lastF = carCtl.trackMap->getFrenet(lastPosXY);
            cout << "last_s: " << lastF.s << " last d: " << lastF.d << "\n";
            int closest_wp = carCtl.trackMap->ClosestWaypoint(lastPosXY, carCtl.trackMap->waypoints_x,
                                                              carCtl.trackMap->waypoints_y);
            int next_wp = carCtl.trackMap->NextWaypoint(lastPosXY, carCtl.trackMap->waypoints_x,
                                                        carCtl.trackMap->waypoints_y);
            cout << "closest_wp: " << closest_wp << " next_wp: " << next_wp << "\n";

            for (int i = 0; i < prev_path_size; i++) {
              next_x_vals.push_back(j[1]["previous_path_x"][i]);
              next_y_vals.push_back(j[1]["previous_path_y"][i]);
            }

            if (generate_path_size > 25) {
              // PHASE 1
              // Just stupidly drive forward at 11MPH.
              //            for (int i = 1; i <= generate_path_size; i++) {
              //              lastF.s += .1;
              //              cout << "new s: " << lastF.s << "\n";
              //              Position posXY = carCtl.trackMap->getXY(lastF);
              //              cout << "new_x: " << posXY.x << " new_y: " << posXY.y << "\n";
              //              double dist = distance(posXY.x, posXY.y, lastPosXY.x, lastPosXY.y);
              //
              //              next_x_vals.push_back(posXY.x);
              //              next_y_vals.push_back(posXY.y);
              //            }
              // Use the proper trajectory generator.

              // PHASE 2
//              Vehicle new_vpt;
//              //other_vehicle_predictions = {};
//              double last_s = -1;
//              for (int i = 1; i <= generate_path_size; i++) {
//                new_vpt = carCtl.get_lane_kinematic(carCtl.last_path_vehicle, 1, i/50., 50., other_vehicle_predictions);
//                auto new_f = carCtl.trackMap->getFrenet(new_vpt.position());
//                if (last_s > 0 && last_s > new_f.s) {
//                  cout << "ERROR\n";
//                  continue;
//                }
//                last_s = new_f.s;
//                cout << i << "-----------\n";
//                cout << "new s: " << new_f.s << "new d: " << new_f.d <<"\n";
//                cout << "new_x: " << new_vpt.x() << " new_y: " << new_vpt.y() << " new_yaw:" << new_vpt.yaw() << "\n";
//                cout << "new_v: " << new_vpt.v() << " new_a: " << new_vpt.a() << "\n";
//                next_x_vals.emplace_back(new_vpt.x());
//                next_y_vals.emplace_back(new_vpt.y());
//              }
//              carCtl.last_path_vehicle = new_vpt;
//            }
              // Choose the next state based on the trajectories of the other cars.
              VehicleController lastCarCtl{carCtl.last_path_vehicle, carCtl.fsm, carCtl.trackMap};
              lastCarCtl.vehicle = carCtl.last_path_vehicle;

              std::pair<fsm::STATE, vector<Vehicle>> best_state_path =
                  planner.choose_next_state(lastCarCtl, other_vehicle_predictions);

              lastCarCtl.fsm->state = best_state_path.first;

              cout << best_state_path.first << "\n";

              //lastCarCtl.extend_trajectory(best_state_path.second);
              for (Vehicle &v: best_state_path.second) {
                next_x_vals.push_back(v.x());
                next_y_vals.push_back(v.y());
                cout << "new_x: " << v.x() << " new_y: " << v.y() << " new_yaw:" << v.yaw() << "\n";
                carCtl.last_path_vehicle = v;
              }
            }
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);



          counter -= 1;

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });








  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string str = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(str.data(), str.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
