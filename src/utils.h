#
//
// Created by Carey, Frank, Sr. on 2/17/18.
//

#ifndef UTILS_H
#define UTILS_H


#include <vector>
#include <string>
#include <cmath>
#include "spline.h"
#include "trajectory.h"

using namespace std;

namespace utils {


  class Position {

  public:

    // X in meters
    double x;

    // Y in meters
    double y;

    //yaw in radians!
    double yaw;

    Position();

    Position(double x, double y);

    Position(double x, double y, double yaw);

    Position convert_to_frame(Position ref_pt);

    Position convert_from_frame(Position ref_pt);

    Position clone();

  };

  class FrenetPos {
  public:

    // s in meters
    double s;

    // d in meters
    double d;

    FrenetPos();

    FrenetPos(double s, double d);

    FrenetPos clone();

  };

  class Spline {
  public:

    explicit Spline(vector<Position> way_pts);
    Position interpolate(double x);

  private:

    tk::spline spline;

  };

  class Map {

  public:
    Map()=default;
    explicit Map(string map_file, double max_s, double speed_limit_mph, double n_lanes);

    double max_s;
    double speed_limit_mph; // in miles per hour.
    double n_lanes;
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    vector<double> interpolated_waypoints_x;
    vector<double> interpolated_waypoints_y;
    vector<double> interpolated_waypoints_s;
    vector<double> interpolated_waypoints_dx;
    vector<double> interpolated_waypoints_dy;

    int ClosestWaypoint(Position pt, vector<double> wp_x, vector<double> wp_y);

    vector<vector<double>> get_near_cars(double s, double speed, double time_horizon, vector<vector<double>> sensor_fusion);

    int NextWaypoint(Position pt, vector<double> wp_x, vector<double> wp_y);

    FrenetPos getFrenet(Position pt);

    Position getXY(FrenetPos frenet);

    int getFrenetLane(FrenetPos frenet);

    int getXYLane(Position position);

    bool is_lane_available(int lane);

    double speed_limit(); // in meters per second.

    Position position_at(Position pos, double timedelta);

    void update_local_waypoints(Position pt, int wp_behind = 2, int wp_ahead = 2);

    //Spline Map::getLocalSpline(Position pt, int n_points);

    bool collision_test(Trajectory2D carTrajectory, vector<double> other_car, double time_limit);

    double s_relative_to(double s, double relative_to);
  };

  constexpr double pi() { return M_PI; }

  double deg2rad(double x);

  double rad2deg(double x);

  string hasData(string s);

  double distance(double x, double y);
  double distance(double x1, double y1, double x2, double y2);

  vector<Position> generate_spline_path(double car, double target_d, double yaw, double velocity,
                       double acceleration,
                       vector<double> &previous_path_x, vector<double> &previous_path_y,
                       vector<double> &next_x_vals, vector<double> &next_y_vals,
                       vector<double> &map_waypoints_s, vector<double> &map_waypoints_x,
                       vector<double> &map_waypoints_y);

  double from_mph(double mph);

  double to_mph(double mph);

  double normalize_rad(double deg);

  double normalize_deg(double deg);

  struct XYList {
    vector<double> x_list ;
    vector<double> y_list ;
  };

  vector<double> parabolicGetXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

}

#endif //UTILS_H

