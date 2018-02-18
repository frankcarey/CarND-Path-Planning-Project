#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "vehicle.h"
#include "utils.h"


using namespace std;
using namespace std::chrono;

// for convenience
using json = nlohmann::json;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	double s;
  	double d_x;
  	double d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  const float MAX_LEGAL_VELOCITY = 100.5; // TODO this is actually 50.

  // Setup details our vehicle needs.
  vector<float> road_data{
      0, //target_speed default to zero initially (in mph!)
      3,  //lanes_available
      -1, //target_s (disabled at first because we don't care.)
      1, //target_lane (starting in lane 1);
      5 * (float) 0.44704, //max_acceleration (in meters per second squared)
      MAX_LEGAL_VELOCITY * (float) 0.44704 // We'll make sure not to exceed this speed. (convert to meters per second)

  };

  Vehicle car{0, Vehicle::lane_to_d(1), 0, 0, 0, "KL"};  // start in lane 1
  car.configure(road_data);

  time_point <system_clock>last_update = system_clock::now();


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&car,&last_update](uWS::WebSocket<uWS::SERVER> ws, const char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length > 2 && data[0] == '4' && data[1] == '2') {
      pi();
      auto s = hasData(data);

      if (!s.empty()) {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          time_point <system_clock>now = system_clock::now();
          time_point <system_clock>last = last_update;
          long long int time_diff = duration_cast<std::chrono::milliseconds>(now - last).count();
          cout << time_diff<< "ms \n";
          last_update = now;
          // j[1] is the data JSON object

          // Main car's localization Data
          //car.x = j[1]["x"];
          //car.y = j[1]["y"];
          car.s = j[1]["s"];
          car.d = j[1]["d"];
          car.yaw = j[1]["yaw"];
          car.v = (float)(j[1]["speed"]) * (float) 0.44704; // convert speed to meters per second.

          cout << "my v:" << car.v << "\n";
          cout << "my a:" << car.a << "\n";
          cout << "my d:" << car.d << "\n";
          cout << "my lane:" << car.get_lane() << "\n";

          // Quick sanity check to kill the program when this happens.
          if (car.get_lane() < 0 ) {
            cout << "you've killed us!!" << "\n";
            exit(1);
          }


          // Previous path data given to the Planner
          vector<float> previous_path_x = j[1]["previous_path_x"];
          vector<float> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

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
          float velocity;
          float vx;
          float vy;
          for (auto &vehicle_data : sensor_fusion) {
            vx = vehicle_data[3];
            vy = vehicle_data[4];
            velocity = sqrt(vx * vx + vy * vy);

            other_vehicles.emplace_back(Vehicle{
                vehicle_data[5], // s
                vehicle_data[6], // d
                velocity, // v
                //sensor_fusion[i][1], // a (assume zero)
                //sensor_fusion[i][1], // yaw (assume zero)
                //sensor_fusion[i][1] // state (assume default)

            });
          }

          json msgJson;

          // The previous path that the car was driving.
          auto prev_size = previous_path_x.size();


          // START BEHAVIOR PLANNING
          if (prev_size > 0) {
            // TODO: This is probably going to cause issues where we need to compare the car against other car's current s?
            car.s = (float) end_path_s;
          }

          bool too_close = false;
          bool stopped = false;

          //cout << other_vehicles.size() << " Cars I can see\n";
          // find the ref_v to use by checking the other cars in our lane from sensor fusion.
          for (auto &other_vehicle : other_vehicles) {
            // check only car is in my lane.
            if (car.in_my_lane(other_vehicle)) {
              float check_car_s = other_vehicle.s;
              check_car_s += ((double) prev_size * .02 *
                  other_vehicle.v); //if using previous points, project car's s value out in time.
              // check that the s value is greater than mine and s gap.
              //cout << (check_car_s - car.s) << "\n";
              if ((check_car_s > car.s) && ((check_car_s - car.s) < 30)) {
                cout << (check_car_s - car.s) << ": Close car!!\n";
                too_close = true;
              }
            } else {
              //cout << i << " : " << other_vehicles[i].s << other_vehicles[i].d << "\n";
            }
          }
//
//          if (too_close) {
//            cout << "TOO CLOSE\n";
//            if (car.v > .224) {
//              // slow down by about 5 m/s
//              // TODO: Video says this can be more efficient if placed when creating the actual points instead of the same
//              // value for all the points.
//              // TODO: This value also seems to high on my machine (car speeds up and slows down more quickly than in the video.
//              car.v -= .224;
//              //                  if (car.get_lane() > 0 ) {
//              //                    // TODO: This is simply making a lane change 30 meters ahead (spline lib and 30M waypoints make it pretty smooth.
//              //                    // TODO: Also we should check to make sure if there are any cars in that lane that are too near to make the change safely.
//              //                    // TODO: We should consider lanes to the left and right, not just hug the left lane.
//              //                    // TODO: Predict where cars will be in the future and set a cost function for what the optimal state for our car will be.
//              //                    // note: simulator operates at 50 samples per second.
//              //                    car.d = Vehicle::lane_to_d(0);
//              //                  }
//            } else {
//              cout << "Stuck in traffic \n";
//            }
//          }

          // If we're not too close and going slower than our goal, speed up.
          if (!too_close && (car.v < car.max_legal_speed)) {
            // speed up by about 5 m/s
            car.accelerate();
            cout << "MOR POWER!!\n";
          } else if (car.v < 0.1) {
            stopped = true;
            car.v = 1;
            car.a = .001;
            cout << "STOPPED\n";
          } else if (too_close || (car.v > car.max_legal_speed)) {
            car.decelerate();
            cout << "SLOW YER ROLE!!\n";
          }

          if (too_close) {
            // Switch lanes.
            cout << "TOO CLOSE!!\n";

          }

          if (car.v < 20 && car.a < 0) {
            // do not allow reverse!
            car.a = .001;
            cout << "NO REVERSE!\n";
          }

          vector<float> next_x_vals;
          vector<float> next_y_vals;

          //generate_path_spline(car, previous_path_x, previous_path_y, next_x_vals, next_y_vals);
          generate_spline_path(car.s, car.d, Vehicle::lane_to_d(car.target_lane), car.yaw, car.v, car.a, previous_path_x, previous_path_y,
            next_x_vals, next_y_vals, map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
