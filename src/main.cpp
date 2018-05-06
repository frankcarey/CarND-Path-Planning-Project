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

  Traj sTrajectory; // trajectory along s
  Traj dTrajectory; //trajectory along d

  h.onMessage([&carCtl, &max_s, &sTrajectory, &dTrajectory](uWS::WebSocket<uWS::SERVER> ws, const char *data, size_t length,
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


          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          auto planner = PathPlanner();

          // j[1] is the data JSON object
          // Main car's localization Data
          cout << "simulator| x: " << x << " y:" << y << " s: " << s << " d: " << d << " deg: " << yaw_deg << " rad: "
               << yaw_radians << "\n";

          carCtl.update(s, d, yaw_radians, speed_mph, j[1]["previous_path_x"].size());

          int lane_desired = (int) floor(d/4);
          double time_horizon = (carCtl.size_horizon - 1) * 0.02; // seconds for time horizon of path
          double time_plan =  (carCtl.size_plan -1) * 0.02; // seconds between each path plannings
          vector<vector<double>> near_cars;
          double pos_x;
          double pos_y;
          double angle;
          double next_s; // next s value
          double next_d; // next d value
          vector<double> sxy;

          vector<double> s_conds; // boundary conditions for s
          vector<double> d_conds; // boundary conditions for d

          vector<Traj> sSet;
          vector<Traj> dSet;
          vector<double> max_min;
          vector<combiTraj> combSet;

          double t_s = (carCtl.size_horizon - 1) * 0.02;
          double t_d = (carCtl.size_horizon - 1) * 0.02;
          double time_manouver = t_s;

          int mct_idx;
          int path_steps_used = (carCtl.size_horizon - carCtl.prev_path_size);

          // CURRENT PATH IS VALID UNTIL NEXT PLAN SO CHECK TIME ELAPSED FROM PREVIOUS PLANNING
          if (path_steps_used < carCtl.size_plan) {
            // NO PLANNING BECAUSE LAST PATH IS NOT EXPIRED
            for (int i = 0; i < carCtl.prev_path_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
          } else {

            if (carCtl.prev_path_size == 0) { // intialize first path
              // set intial s and d conditions
              s_conds = {s, 0, 0, carCtl.speed_limit, 0};
              d_conds = {d, 0, 0, 0 * 4. + 2., 0, 0};

            } else {
              // prepare new boundary conditions
              s_conds.clear();
              d_conds.clear();
              double last_time = path_steps_used * 0.02;

              double ss_i = sTrajectory.getDis(last_time); // s position at time t_i
              while (ss_i > max_s) {
                ss_i -= max_s;
              }
              double vs_i = sTrajectory.getVel(last_time); // velocity along s at time t_ti
              double as_i = sTrajectory.getAcc(last_time); // acceleration along s at time t_i

              double dd_i = dTrajectory.getDis(last_time); // d position at time t_i
              double vd_i = dTrajectory.getVel(last_time); // velocity along d at time t_ti
              double ad_i = dTrajectory.getAcc(last_time); // acceleration along d at time t_i

              // push conditions
              s_conds = {ss_i, vs_i, as_i, carCtl.speed_limit, 0};
              d_conds = {dd_i, vd_i, ad_i, lane_desired * 4. + 2., 0, 0};
            }

            near_cars = carCtl.trackMap->get_near_cars(s, speed_mph, time_horizon, j[1]["sensor_fusion"]);

            //generate set of unidimensional trajectories
            combSet = planner.generate_trajectories(carCtl, s_conds, d_conds, time_horizon, lane_desired, near_cars);

            // continue to generate trajectories if no suitable trajectory is found, until time_maneuver is lower than 2 seconds
            while (combSet.empty() && time_manouver > 2.) {

              //generate set of unidimensional trajectories
              combSet = planner.generate_trajectories(carCtl, s_conds, d_conds, time_manouver, lane_desired, near_cars);

              // find minimal cost trajectory
              if (~combSet.empty()) {
                time_manouver *= 0.9; // if no trajectory is found, repeate trajectories generation with a smaller time horizon
              }
            }
            mct_idx = planner.minimal_cost_trajectory(combSet);

            sTrajectory = combSet[mct_idx].Trs;  // set s trajectory
            dTrajectory = combSet[mct_idx].Trd; // set d trajectory

            cout << "next_s" << next_s << "\n";
            //generate next points using selected trajectory with a time pace of 0.02 seconds
            for (int i = 0; i < carCtl.size_horizon; i++) {
              next_s = sTrajectory.getDis(i * 0.02); // get s value at time i*0.02
              next_d = dTrajectory.getDis(i * 0.02); // get d value at time d*0.02

              // convert  to  global coordinates
              sxy = utils::parabolicGetXY(next_s, next_d, carCtl.trackMap->waypoints_s,
                                          carCtl.trackMap->waypoints_x, carCtl.trackMap->waypoints_y);

              // pass to simulator
              next_x_vals.push_back(sxy[0]);
              next_y_vals.push_back(sxy[1]);
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
