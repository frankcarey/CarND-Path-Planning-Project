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

  class Spline {
  public:

    explicit Spline(vector<Point> way_pts);
    Point interpolate(double x);

  private:

    tk::spline spline;

  };

  class Map {
  public:
    explicit Map(string map_file, double max_s);

    double max_s;
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;



  };

  constexpr double pi() { return M_PI; }

  double deg2rad(double x);

  double rad2deg(double x);

  string hasData(string s);

  double distance(double x1, double y1, double x2, double y2);

  int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

  int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  vector<double>
  getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  Point
  getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

  void
  generate_spline_path(double current_s, double current_d, double target_d, double yaw, double velocity,
                       double acceleration,
                       vector<double> &previous_path_x, vector<double> &previous_path_y,
                       vector<double> &next_x_vals, vector<double> &next_y_vals,
                       vector<double> &map_waypoints_s, vector<double> &map_waypoints_x,
                       vector<double> &map_waypoints_y);

  double from_mph(double mph);

  double to_mph(double mph);

  double normalize_rad(double deg);

  double normalize_deg(double deg);
}

#endif //UTILS_H

