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
  const double MAX_LEGAL_MPH = 100.5; // in MPH! TODO this is actually 50.
  const int NUMBER_OF_LANES = 3;
  const double MAX_ACCELERATION = 8; // in meters per second.



  utils::Map trackMap =  utils::Map(map_file, max_s, MAX_LEGAL_MPH, NUMBER_OF_LANES);


  Vehicle car{};
  VehicleFSM fsm{};

  //car.configure(road_data);
  VehicleController carCtl = VehicleController(car, &fsm, &trackMap);
  // TODO: Initialize this initial position in a better way.
  //carCtl.vehicle = Vehicle(0, Position(909.47, 1128.67));
  carCtl.max_acceleration = MAX_ACCELERATION;

  bool initialized = false;

  h.onMessage([&carCtl, &initialized](uWS::WebSocket<uWS::SERVER> ws, const char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = utils::hasData(data);

      if (!s.empty()) {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {

          json msgJson;

          //cout << j.dump() << " : JSON\n";

          double x = j[1]["x"];
          double y = j[1]["y"];
          double yaw_radians = utils::deg2rad(j[1]["yaw"]);
          double speed_mph = j[1]["speed"];
          double s_ = j[1]["s"];
          double d = j[1]["d"];


          // j[1] is the data JSON object
          // Main car's localization Data
          carCtl.update(x, y, yaw_radians, speed_mph, j[1]["previous_path_x"].size());
          PathPlanner

          // Just return so we get a previous state.
          if(!initialized) {
            initialized = true;
          } else {


            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];


            auto future_carCtl = VehicleController(carCtl.vehicle, &fsm, &trackMap);

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            //["sensor_fusion"] A 2d vector of cars and then that car's [
            // * [0] car's unique ID
            // * [1] car's x position in map coordinates
            // * [2] car's y position in map coordinates
            // * [3] car's x velocity in m/s
            // * [4] car's y velocity in m/s
            // * [5] car's s position in frenet coordinates
            // * [6] car's d position in frenet coordinates.
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

            // Choose the next state based on the trajectories of the other cars.
            std::pair<fsm::STATE, vector<Vehicle>> best_state_path = carCtl.choose_next_state(
                other_vehicle_predictions);

            carCtl.fsm->state = best_state_path.first;

            carCtl.extend_trajectory(best_state_path.second);
          }

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (Vehicle &point: carCtl.trajectory) {
            next_x_vals.emplace_back(point.x());
            next_y_vals.emplace_back(point.y());
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

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
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
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
