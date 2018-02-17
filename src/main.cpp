#include <fstream>
#include <cmath>
#include "spline.h"
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
      0, //target_speed default to zero initially.
      3,  //lanes_available
      -1, //target_s (disabled at first because we don't care.)
      -1, //target_lane (disabled at first because we don't care.);
      5, //max_acceleration
      MAX_LEGAL_VELOCITY, // We'll make sure not to exceed this speed.

  };

  Vehicle car{0, Vehicle::lane_to_d(1), 0, 0, 0, "KL"};  // start in lane 1
  car.target_lane = 1; // stay in lane 1 for now.
  car.configure(road_data);

  // Start in lane 1 (middle lane)
  //int lane = 1;
  // Set a reference velocity to the target (next waypoint).
  // Start it at zero, so we don't accelerate instantaneously.
  //double ref_vel = 0; //mph

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

      // check that the s value is greater than mine and s gap.
//      cout << (check_car_s - car.s) << "\n";
//      if ((check_car_s > car.s) && ((check_car_s - car.s) < 30)) {
//        cout << (check_car_s - car.s) << "\n";
//        too_close = true;

      if (s != "") {
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
          car.v = j[1]["speed"];

          cout << "my d:" << car.d << "\n";
          cout << "my lane:" << car.get_lane() << "\n";

          if (car.get_lane() < 0 ) {
            cout << "you've killed us!!" << "\n";
            exit(1);
          }


          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
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

          //cout << sensor_fusion.size() << " Cars I can see\n";

          vector<Vehicle> other_vehicles;
          float velocity;
          float vx;
          float vy;
          for (int i = 0; i < sensor_fusion.size(); i++) {
            vx = sensor_fusion[i][3];
            vy = sensor_fusion[i][4];
            velocity = sqrt(vx * vx + vy * vy);

            other_vehicles.emplace_back(Vehicle{
                sensor_fusion[i][5], // s
                sensor_fusion[i][6], // d
                velocity, // v
                //sensor_fusion[i][1], // a (assume zero)
                //sensor_fusion[i][1], // yaw (assume zero)
                //sensor_fusion[i][1] // state (assume default)
            });
          }

          json msgJson;

          // The previous path that the car was driving.
          int prev_size = previous_path_x.size();


          // START BEHAVIOR PLANNING
          if (prev_size > 0) {
            //car.s = (float) end_path_s;
          }

          bool too_close = false;
          bool stopped = false;

          //cout << other_vehicles.size() << " Cars I can see\n";
          // find the ref_v to use by checking the other cars in our lane from sensor fusion.
          for (int i = 0; i < other_vehicles.size(); i++) {
            // check only car is in my lane.
            if (car.in_my_lane(other_vehicles[i])) {
              float check_car_s = other_vehicles[i].s;
              check_car_s += ((double) prev_size * .02 *
                              other_vehicles[i].v); //if using previous points, project car's s value out in time.
              // check that the s value is greater than mine and s gap.
              cout << (check_car_s - car.s) << "\n";
              if ((check_car_s > car.s) && ((check_car_s - car.s) < 30)) {
                cout << (check_car_s - car.s) << "\n";
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
          if (!too_close && (car.target_speed < GOAL_VELOCITY)) {
            // speed up by about 5 m/s
            car.target_speed += .224;
            cout << "MOR POWER!!\n";
          } else if (car.target_speed < .224) {
            stopped = true;
            cout << "STOPPED\n";
          } else if (too_close || car.target_speed > GOAL_VELOCITY) {
            car.target_speed = GOAL_VELOCITY;
            cout << "SLOW YER ROLE!!\n";
          }

          if (too_close) {
            // Switch lanes.
            cout << "TOO CLOSE!!\n";

          }



          vector<double> next_x_vals;
          vector<double> next_y_vals;

          if (!stopped) {
            // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
            // Later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
            vector<double> way_pts_x; // ptsx in video
            vector<double> way_pts_y; // ptsy in video

            // Reference x, y, and yaw states. Either we will reference the starting point as where the car is or
            // at the previous paths end point.
            vector<double> car_xy = getXY(car.s, car.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            double ref_x = car_xy[0];
            double ref_y = car_xy[1];
            double ref_yaw = deg2rad(car.yaw);

           // cout << ref_x << ":" << ref_y << ":" << ref_yaw << "\n";

            // If the previous points are almost empty, use the car as a starting reference
            // and infer two points based on the car's current position and heading.
            if (prev_size < 2) {
              // Use the two points that make the path tangent to the car.
              double prev_car_x = ref_x - cos(car.yaw);
              double prev_car_y = ref_y - sin(car.yaw);
              double car_x = ref_x;
              double car_y = ref_y;

              way_pts_x.push_back(prev_car_x);
              way_pts_x.push_back(car_x);

              way_pts_y.push_back(prev_car_y);
              way_pts_y.push_back(car_y);

            } else {
              // Otherwise, use the previous path's endpoint as a starting reference.

              // Redefine the reference state as the previous path endpoint.
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Use the two points that make the path tangent to previous path's end point.
              way_pts_x.push_back(ref_x_prev);
              way_pts_x.push_back(ref_x);

              way_pts_y.push_back(ref_y_prev);
              way_pts_y.push_back(ref_y);

            }

            // In Frenet, add evenly 60m spaced points ahead of the starting reference. (30 gave errors sometimes)
            vector<double> next_wp0 = getXY(car.s + 60, Vehicle::lane_to_d(car.target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car.s + 120, Vehicle::lane_to_d(car.target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car.s + 180, Vehicle::lane_to_d(car.target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            way_pts_x.push_back(next_wp0[0]);
            way_pts_x.push_back(next_wp1[0]);
            way_pts_x.push_back(next_wp2[0]);

            way_pts_y.push_back(next_wp0[1]);
            way_pts_y.push_back(next_wp1[1]);
            way_pts_y.push_back(next_wp2[1]);

            // Shift to the car's coordinates to make the math easier later.
            for (int i = 0; i < way_pts_x.size(); i++) {

              // Shirt the car reference angle to 0 degrees.
              double shift_x = way_pts_x[i] - ref_x;
              double shift_y = way_pts_y[i] - ref_y;

              way_pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
              way_pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

            }

            // Create a spline.
            tk::spline spline;

            // Set x,y coordinates as anchor points of the spline.
            spline.set_points(way_pts_x, way_pts_y);

            // Define the actual x,y points we'll be using for the planner.
            for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);

            }

            double target_x = 30.;
            double target_y = spline(target_x);
            double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

            double x_add_on = 0;

            // Fill up the rest of our path planner after filling it with previous points,
            // here we will always output 50 points.
            for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

              double N = (target_dist / (.02 * car.target_speed / 2.24));
              double x_point = x_add_on + (target_x) / N;
              double y_point = spline(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // Rotate back to map coordinates.
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
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
