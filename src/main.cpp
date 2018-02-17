#include <fstream>
#include <math.h>
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

using namespace std;
using namespace std::chrono;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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

  // Setup our vehicle.
  vector<int> road_data{
      0, //target_speed default to zero initially.
      3,  //lanes_available
      -1, //goal_s (disabled at first because we don't care.)
      -1, //goal_lane (disabled at first because we don't care.);
      5, //max_acceleration
  };



  Vehicle car{0, Vehicle::lane_to_d(1), 0, 0, 0, "CL"}; //start in lane 1;
  car.configure(road_data);

  // Start in lane 1 (middle lane)
  //int lane = 1;
  // Set a reference velocity to the target (next waypoint).
  // Start it at zero, so we don't accelerate instantaneously.
  //double ref_vel = 0; //mph
  const float GOAL_VELOCITY = 49.5;

  time_point <system_clock>last_update = system_clock::now();


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&car,&GOAL_VELOCITY,&last_update](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

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

          vector<Vehicle> other_vehicles;
          float velocity;
          float vx;
          float vy;
          for (int i; i < sensor_fusion.size(); i++) {
            vx = sensor_fusion[i][3];
            vy = sensor_fusion[i][4];
            velocity = sqrt(vx * vx + vy * vy);

            other_vehicles.push_back(Vehicle{
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

          // find the rev_v to use by checking the other cars in our lane from sensor fusion.
//          for (int i = 0; i < other_vehicles.size(); i++) {
//            // check only car is in my lane.
//            if (car.in_my_lane(other_vehicles[i])) {
//              float check_car_s = other_vehicles[i].s;
//              check_car_s += ((double) prev_size * .02 *
//                              other_vehicles[i].v); //if using previous points, project car's s value out in time.
//              // check that the s value is greater than mine and s gap.
//              if ((check_car_s > car.s) && ((check_car_s - car.s) < 30)) {
//
//                // TODO: Do some logic here to handle a car in our way.
//
//                // lower velocity so we don't crash into them.
//                //ref_vel = 29.5; // TODO: Make this smarter.
//                too_close = true;
//              }
//            }
//          }
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
          } else if (car.target_speed > GOAL_VELOCITY) {
            car.target_speed = GOAL_VELOCITY;
            cout << "SLOW YER ROLE!!\n";
          }
          cout << car.target_speed << "\n";


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

            // In Frenet, add evenly 30m spaced points ahead of the starting reference.
            vector<double> next_wp0 = getXY(car.s + 30, car.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car.s + 60, car.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car.s + 90, car.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

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
