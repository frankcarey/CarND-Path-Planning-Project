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

using namespace std;

namespace utils {


  class Point {

  public:

    // X in meters
    double x;

    // Y in meters
    double y;

    //yaw in radians!
    double yaw;

    Point();

    Point(double x, double y);

    Point(double x, double y, double yaw);

    Point convert_to_frame(Point ref_pt);

    Point convert_from_frame(Point ref_pt);

    Point clone();

  };

  class Frenet {
  public:

    // s in meters
    double s;

    // d in meters
    double d;

    Frenet();

    Frenet(double s, double d);

  };

  class Spline {
  public:

    explicit Spline(vector<Point> way_pts);
    Point interpolate(double x);

  private:

    tk::spline spline;

  };

  class Map {
  public:
    Map()=default;
    explicit Map(string map_file, double max_s, double speed_limit_mph, double n_lanes);

    double max_s;
    double speed_limit_mph;
    double n_lanes;
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    int ClosestWaypoint(Point pt);

    int NextWaypoint(Point pt);

    Frenet getFrenet(Point pt);

    Point getXY(Frenet frenet);

    bool is_lane_available(int lane);

  };

  constexpr double pi() { return M_PI; }

  double deg2rad(double x);

  double rad2deg(double x);

  string hasData(string s);

  double distance(double x1, double y1, double x2, double y2);

  vector<Point> generate_spline_path(double car, double target_d, double yaw, double velocity,
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
}

#endif //UTILS_H

