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

bool funcia (combiTraj i, combiTraj j) { return (i.Cost < j.Cost); } // small helper function to sort a set of combiTraj by their costs

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
  // TODO: Initialize this initial position in a better way.
  //carCtl.vehicle = Vehicle(0, Position(909.47, 1128.67));

  bool initialized = false;

  int counter = 0;
  int size_prev_plan = 0; // number of points already reached since last planned path
  int size_prev_path = 0; // total size of last path passed to simulator
  int size_kept = 0; // points of previous path added to current path
  double speed_goal = 10e2; //desired speed
  Traj longTrajectory; // trajectory along s
  Traj lateralTrajectory; //trajectory along d

  h.onMessage([&carCtl, &initialized, &counter, &size_prev_plan, &size_prev_path, &size_kept, &max_s, &speed_goal,
                  &longTrajectory, &lateralTrajectory](uWS::WebSocket<uWS::SERVER> ws, const char *data, size_t length,
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
          // TODO: uncommment this and fix the interpolated waypoints. they seem to break near
          // new waypoints, probably because the map is wrong?
          //carCtl.trackMap->update_local_waypoints(carCtl.vehicle.position(), 2, 2);

          //cout << "post-update| x: " << carCtl.vehicle.x() << " y:" << carCtl.vehicle.y() << " yaw:"
          //     << carCtl.vehicle.yaw() << "\n";
          //FrenetPos carF = carCtl.trackMap->getFrenet(carCtl.vehicle.position());
          //cout << "post-update-frenet| s: " << carF.s << " d:" << carF.d << "\n";
          //Position carPos = carCtl.trackMap->getXY(carF);
          //cout << "post-update| x: " << carPos.x << " y:" << carPos.y << " yaw:" << carPos.yaw << "\n";

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
              Vehicle new_vehicle{vehicle_data[0], FrenetPos{vehicle_data[5], vehicle_data[6]}};
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
            int generate_path_size = 50 - prev_path_size;
            cout << "generate_path_size: " << generate_path_size << "\n";

              int lane_desired = (int) floor(d/4);
              const int size_plan = 10; // size of path already driven after which a new path must be planned
              const int size_keep = 0; // points of previous path to add to new path


              double time_horizon = (carCtl.size_horizon - 1) * 0.02; // seconds for time horizon of path
              double time_plan =  (size_plan -1) * 0.02; // seconds between each path plannings
              speed_goal = min(carCtl.speed_limit, speed_goal); // desired velocity for car
              std::vector<std::vector<double>> near_cars;
              double pos_x;
              double pos_y;
              double angle;
              int path_size = prev_path_size; // how many points where passed in previous plan

              double next_s; // next s value
              double next_d; // next d value
              vector<double> sxy;

              std::vector<double> conds; // boundary conditions for s
              std::vector<double> d_conds; // boundary conditions for d


              // Choose the next state based on the trajectories of the other cars.
              // select the nearest cars from sensor fusion data
              for (int i=0; i< sensor_fusion.size(); i++)
              {
                double sc = sensor_fusion[i][5];
                // If the car is further than half way round the track from us, then measure from the other direction.
                if (abs(s - sc) > max_s/2.){
                  if (s > max_s/2.) {
                    sc += max_s;
                  } else {
                    sc -= max_s;
                  }
                }
                double dc = sensor_fusion[i][6];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double vc = sqrt(vx*vx + vy*vy);

                if (s > sc) // car is behind us
                {
                  // check if the car is at a distance such that it can reach our car in time_horizon*0.7 seconds at current speed
                  if (vc*time_horizon + sc >= s + speed_mph*0.44704*time_horizon*0.7 - 4.)
                  {
                    near_cars.push_back(sensor_fusion[i]);
                  }
                }
                else //car is ahead of us
                {
                  // check if the car is at a distance that our car can reach in time_horizon seconds at max speed, even if the car decelerates to 1/3 of its current velocity
                  if (vc*time_horizon/3. + sc <= s + speed_mph*0.44704*time_horizon + 4.)
                  {
                    near_cars.push_back(sensor_fusion[i]);
                  }
                }
              }

              if(prev_path_size == 0) // intialize first path
              {
                // set intial s and d conditions
                conds = {s, 0, 0, speed_goal, 0};
                d_conds = {d, 0, 0, 0 * 4. + 2., 0, 0};

                vector<Traj> longSet;
                vector<Traj> lateSet;
                vector<double> max_min;

                //generate set of unidimensional trajectories
                vector<combiTraj> combSet = planner.generate_trajectories(conds, d_conds, time_horizon, speed_goal,
                                                                           lane_desired,
                                                                           {carCtl.speed_limit, carCtl.acc_limit, carCtl.jerk_limit}, near_cars);

                // find minimal cost trajectory
                double min_Comb_Cost = 10e10;
                int min_Comb_idx = 0;
                for (int k = 0; k < combSet.size(); k++) {

                  if (min_Comb_Cost > combSet[k].Cost) {
                    min_Comb_Cost = combSet[k].Cost;
                    min_Comb_idx = k;
                  }
                }

                longTrajectory = combSet[min_Comb_idx].Trs;  // set s trajectory
                lateralTrajectory = combSet[min_Comb_idx].Trd; // set d trajectory

                size_prev_path = 0;

                //generate next points using selected trajectory with a time pace of 0.02 seconds
                for (int i = 0; i < carCtl.size_horizon; i++) {
                  next_s = longTrajectory.getDis(i * 0.02); // get s value at time i*0.02
                  next_d = lateralTrajectory.getDis(i * 0.02); // get d value at time d*0.02

                  // convert  to  global coordinates
                  sxy = utils::parabolicGetXY(next_s, next_d, carCtl.trackMap->waypoints_s,
                                              carCtl.trackMap->waypoints_x, carCtl.trackMap->waypoints_y);

                  // pass to simulator
                  next_x_vals.push_back(sxy[0]);
                  next_y_vals.push_back(sxy[1]);

                  size_prev_path++;
                }
                size_kept = 0;
              }
              else {
                // CURRENT PATH IS VALID UNTIL NEXT PLAN SO CHECK TIME ELAPSED FROM PREVIOUS PLANNING

                size_prev_plan = size_prev_path - path_size;

                if (size_prev_plan >= size_plan) {

                  // PLAN AGAIN AND RESET time_prev_path

                  size_prev_path = 0;

                  // KEEP points of previous path
                  for (int i = 0; i < size_keep; i++) {
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);

                    size_prev_path++;

                  }

                  // prepare new boundary conditions
                  conds.clear();
                  d_conds.clear();
                  // point from wich to start new plan
                  int point_i = size_prev_plan + size_keep - size_kept + 1;
                  size_kept = size_keep;
                  double t_i = (point_i - 1) * 0.02;


                  double ss_i = longTrajectory.getDis(t_i); // s position at time t_i
                  while (ss_i > max_s) {
                    ss_i -= max_s;
                  }
                  double vs_i = longTrajectory.getVel(t_i); // velocity along s at time t_ti
                  double as_i = longTrajectory.getAcc(t_i); // acceleration along s at time t_i

                  double dd_i = lateralTrajectory.getDis(t_i); // d position at time t_i
                  double vd_i = lateralTrajectory.getVel(t_i); // velocity along d at time t_ti
                  double ad_i = lateralTrajectory.getAcc(t_i); // acceleration along d at time t_i

                  // push conditions
                  conds = {ss_i, vs_i, as_i, speed_goal, 0};
                  d_conds = {dd_i, vd_i, ad_i, lane_desired * 4. + 2., 0, 0};

                  double t_s = (carCtl.size_horizon - size_keep - 1) * 0.02;
                  double t_d = (carCtl.size_horizon - size_keep - 1) * 0.02;

                  vector<combiTraj> combSet; // set of combined trajectories
                  double time_manouver = t_s;
                  int min_Comb_idx = 0;

                  // continue to generate trajectories if no suitable trajectory is found, until time_manouver is lower than 2 seconds
                  while (combSet.size() < 1 && time_manouver > 2.) {
                    vector<Traj> longSet;
                    vector<Traj> lateSet;
                    vector<double> max_min;

                    //generate set of unidimensional trajectories
                    combSet = planner.generate_trajectories(conds, d_conds, time_manouver,
                                                                               speed_goal, lane_desired,
                                                                               {carCtl.speed_limit, carCtl.acc_limit, carCtl.jerk_limit}, near_cars);

                    // find minimal cost trajectory
                    if (combSet.size() > 0) {
                      std::sort(combSet.begin(), combSet.end(), funcia);
                    } else {
                      time_manouver *= 0.9; // if no trajectory is found, repeate trajectories generation with a smaller time horizon
                    }
                  }

                  longTrajectory = combSet[0].Trs;
                  lateralTrajectory = combSet[0].Trd;


                  for (int i = 0; i < (carCtl.size_horizon - size_keep); i++) {
                    //generate next points using selected trajectory with a time pace of 0.02 seconds
                    next_s = longTrajectory.getDis(i * 0.02);
                    next_d = lateralTrajectory.getDis(i * 0.02);

                    // convert  to  global coordinates
                    sxy = parabolicGetXY(next_s, next_d, carCtl.trackMap->waypoints_s, carCtl.trackMap->waypoints_x, carCtl.trackMap->waypoints_y);

                    // pass points to simulator
                    next_x_vals.push_back(sxy[0]);
                    next_y_vals.push_back(sxy[1]);

                    size_prev_path++;
                  }

                  size_prev_plan = 0;

                } else {
                  // NO PLANNING BECAUSE LAST PATH IS NOT EXPIRED
                  for (int i = 0; i < path_size; i++) {
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
                  }
                }
              }
            }
          //}

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
