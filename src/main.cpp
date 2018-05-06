#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "trajectory.h"
#include "utils.h"
#include "vehicle.h"
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
  VehicleFSM fsm{fsm::KL, &trackMap};

  //car.configure(road_data);
  VehicleController carCtl = VehicleController(car, &fsm, &trackMap);

  // Keep trajectories between iterations so we can use it
  // to easily calculate our current position, velocity, and acceleration
  // independently for s and d.
  Traj sTrajectory; // trajectory along s
  Traj dTrajectory; //trajectory along d

  h.onMessage([&carCtl, &sTrajectory, &dTrajectory](uWS::WebSocket<uWS::SERVER> ws, const char *data, size_t length,
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

          json msgJson;

          //cout << j.dump() << " : JSON\n";

          double x = j[1]["x"];
          double y = j[1]["y"];
          double yaw_deg = j[1]["yaw"]; // The simulator can send deg > 360!
          double yaw_radians = utils::normalize_rad(utils::deg2rad(yaw_deg));
          double speed_mph = j[1]["speed"];
          double s = j[1]["s"];
          double d = j[1]["d"];

          // We'll return these values back to the simulator.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          auto planner = PathPlanner();

          // j[1] is the data JSON object
          // Main car's localization Data
          cout << "simulator| x: " << x << " y:" << y << " s: " << s << " d: " << d << " deg: " << yaw_deg << " rad: "
               << yaw_radians << "\n";

          carCtl.update(s, d, yaw_radians, speed_mph, j[1]["previous_path_x"].size());
          int path_steps_used = (carCtl.size_horizon - carCtl.prev_path_size);

          int mct_idx; // index of the minimum cost trajectory

          // If the number of path steps used is less than the delay, then pass the same
          // steps back to the simulator. Used for efficiency.
          if (path_steps_used < carCtl.plan_delay) {
            for (int i = 0; i < carCtl.prev_path_size; i++) {
              next_x_vals.push_back(j[1]["previous_path_x"][i]);
              next_y_vals.push_back(j[1]["previous_path_y"][i]);
            }
          } else {

            // We've got to create new points then. If we're starting from scratch,
            // then the trajectories are not valid, so assume we're stopped.
            vector<double> s_state; // s state {s pos, s vel, s acc, s speed, ? }
            vector<double> d_state; // d state {d pos, d vel, d acc, curr lane d, ?}
            if (carCtl.prev_path_size == 0) {

              // Assume we're stopped, so velocity and acceleration are all zero.
              s_state = {s, 0, 0, carCtl.speed_limit, 0};
              d_state = {d, 0, 0, carCtl.get_lane() * 4. + 2., 0, 0};
            } else {

              // Assume we're moving. Find out at which time we're at along our last trajectory.
              double last_time = path_steps_used * 0.02;

              // Use the last saved trajectories to figure out where we are and our
              // velocity and acceleration.
              s = sTrajectory.getDis(last_time);
              while (s > carCtl.trackMap->max_s) {
                s -= carCtl.trackMap->max_s;
              }
              double s_vel = sTrajectory.getVel(last_time); // velocity along s at last_time
              double s_acc = sTrajectory.getAcc(last_time); // acceleration along s at last_time
              d = dTrajectory.getDis(last_time); // d position at last_time
              double d_vel = dTrajectory.getVel(last_time); // velocity along d at last_time
              double d_acc = dTrajectory.getAcc(last_time); // acceleration along d at last_time

              s_state = {s, s_vel, s_acc, carCtl.speed_limit, 0};
              d_state = {d, d_vel, d_acc, carCtl.get_lane() * 4. + 2., 0, 0};
            }

            // How many seconds ahead do we need to calculate a trajectory for?
            double time_horizon = (carCtl.size_horizon - 1) * 0.02;

            // Get the cars that are near by during the time_horizon to test for collisions.
            vector<vector<double>> near_cars = carCtl.trackMap->get_near_cars(s, speed_mph, time_horizon, j[1]["sensor_fusion"]);

            // Generate a set of trajectories and costs using the s and d states.
            vector<combiTraj> combSet = planner.generate_trajectories(carCtl, s_state, d_state, time_horizon, carCtl.get_lane(), near_cars);

            // Rarely, we can't find any trajectory with the full time horizon, like in crowded traffic,
            // so we lower the time_horizon until we do.
            while (combSet.empty() && time_horizon > 2.) {
              time_horizon *= 0.9;
              combSet = planner.generate_trajectories(carCtl, s_state, d_state, time_horizon, carCtl.get_lane(), near_cars);
            }

            // Find the best trajectories in the set with the minimal cost.
            mct_idx = planner.minimal_cost_trajectory(combSet);
            sTrajectory = combSet[mct_idx].Trs;
            dTrajectory = combSet[mct_idx].Trd;

            // Generate x,y coordinates from the best trajectories.
            double next_s;
            double next_d;
            vector<double> xy_vector;
            for (int i = 0; i < carCtl.size_horizon; i++) {
              next_s = sTrajectory.getDis(i * 0.02); // get s value at time i*0.02
              next_d = dTrajectory.getDis(i * 0.02); // get d value at time d*0.02

              // Convert s and d to map coordinates.
              xy_vector = utils::parabolicGetXY(next_s, next_d, carCtl.trackMap->waypoints_s,
                                          carCtl.trackMap->waypoints_x, carCtl.trackMap->waypoints_y);

              next_x_vals.push_back(xy_vector[0]);
              next_y_vals.push_back(xy_vector[1]);
            }
          }

          // Send the x and y values back to the simulator.
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
