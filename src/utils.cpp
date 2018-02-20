#include "utils.h"
#include <sstream>
#include <fstream>


using namespace std;

namespace utils {

  double deg2rad(double x) { return x * pi() / 180; }

  double rad2deg(double x) { return x * 180 / pi(); }

  double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
  string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of('}');
    if (found_null != string::npos) {
      return "";
    } else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 2);
    }
    return "";
  }

  int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); i++) {
      double map_x = maps_x[i];
      double map_y = maps_y[i];
      double dist = utils::distance(x, y, map_x, map_y);
      if (dist < closestLen) {
        closestLen = dist;
        closestWaypoint = i;
      }

    }

    return closestWaypoint;

  }

  int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {

    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = min(2 * utils::pi() - angle, angle);

    if (angle > utils::pi() / 4) {
      closestWaypoint++;
      if (closestWaypoint == maps_x.size()) {
        closestWaypoint = 0;
      }
    }

    return closestWaypoint;
  }

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  vector<double>
  getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
    int next_wp = utils::NextWaypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
      prev_wp = (int) maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = utils::distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
      frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
      frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};

  }

// Transform from Frenet s,d coordinates to Cartesian x,y
  Point
  getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
      prev_wp++;
    }

    auto wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};

  }

  void
  generate_spline_path(double current_s, double current_d, double target_d, double yaw, double velocity, double acceleration,
                       vector<double> &previous_path_x, vector<double> &previous_path_y,
                       vector<double> &next_x_vals, vector<double> &next_y_vals,
                       vector<double> &map_waypoints_s, vector<double> &map_waypoints_x,
                       vector<double> &map_waypoints_y) {

    auto prev_size = previous_path_x.size();

    // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
    // Later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
    vector<Point> way_pts;

    // Reference x, y, and yaw states. Either we will reference the starting point as where the car is or
    // at the previous paths end point.
    Point car_pt = getXY(current_s, current_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    car_pt.yaw = deg2rad(yaw);


    // Use a proxy for the current car position, this will be either where we really are, or the end of the active path.
    Point curr_car_pt = car_pt.clone();

    Point prev_car_pt;
    // If the previous points are almost empty, use the car as a starting reference
    // and infer two points based on the car's current position and heading.
    if (prev_size < 2) {
      // Use the two points that make the path tangent to the car.
      prev_car_pt = Point(curr_car_pt.x - cos(curr_car_pt.yaw), curr_car_pt.y - sin(curr_car_pt.yaw));

      way_pts.push_back(prev_car_pt);
      way_pts.push_back(curr_car_pt);

    } else {
      // Otherwise, use the previous path's endpoint as a starting reference.

      // Redefine the reference state as the previous path endpoint.
      prev_car_pt = Point(previous_path_x[prev_size - 1], previous_path_y[prev_size - 1]);
      curr_car_pt = Point(previous_path_x[prev_size - 2], previous_path_y[prev_size - 2]);
      curr_car_pt.yaw = atan2(curr_car_pt.y - prev_car_pt.y, curr_car_pt.x - prev_car_pt.x);

      way_pts.push_back(prev_car_pt);
      way_pts.push_back(curr_car_pt);

    }

    // In Frenet, add evenly 60m spaced points ahead of the starting reference. (30 gave errors sometimes)
    way_pts.push_back(getXY(current_s + 60, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y));
    way_pts.push_back(getXY(current_s + 120, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y));
    way_pts.push_back(getXY(current_s + 180, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y));


    // Shift to the car's coordinates to make the math easier later.
    for (auto &way_pt : way_pts) {

      // Shift the car reference angle to 0 degrees.
      way_pt = way_pt.convert_to_frame(curr_car_pt);
    }

    Spline spline = Spline(way_pts);

    // Define the actual x,y points we'll be using for the planner.
    // Start by filling the next_x_vals with what's left over from previous path.
    for (int i = 0; i < previous_path_x.size(); i++) {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);

    }

    double target_x = 30.;
    Point pt = spline.interpolate(target_x);
    double target_dist = sqrt(pt.x * pt.x + pt.y * pt.y);

    double x_add_on = 0;

    // Fill up the rest of our path planner after filling it with previous points,
    // here we will always output 50 points.
    // TODO: (I reduced this to 5 as the acceleration kicked in too hard at the end of the first
    // batch of paths..  we should account for acceleration!
    const int INCREMENTS = 50;
    double acceleration_inc = acceleration / INCREMENTS;

    for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
      velocity += acceleration_inc;

      double N = (target_dist / (.02 * velocity));

      double x_point = ((target_x / N) + x_add_on);
      Point next_pt = spline.interpolate(x_point)
          .convert_from_frame(car_pt);

      x_add_on = x_point;

      next_x_vals.push_back(next_pt.x);
      next_y_vals.push_back(next_pt.y);
    }
  }

  double from_mph(double mph) {
    return mph * (double) 0.44704;
  }

  double to_mph(double meters_per_s) {
    return meters_per_s / 0.44704;
  }

  double normalize_deg(double deg) {
    deg = fmod(deg, 360);
    if (deg < 0)
      deg += 360;
    return deg;
  }

  double normalize_rad(double deg) {
    double _2xPI = 2 * M_PI;
    deg = fmod(deg, _2xPI);
    if (deg < 0)
      deg += _2xPI;
    return deg;
  }

  tk::spline create_spline(vector<Point> way_pts) {

    vector<double> x_pts;
    vector<double> y_pts;

    for (auto &way_pt : way_pts) {
      x_pts.push_back(way_pt.x);
      y_pts.push_back(way_pt.y);
    }

    // Create a spline.
    tk::spline spline;

    // Set x,y coordinates as anchor points of the spline.
    spline.set_points(x_pts, y_pts);

    return spline;

  }

  Point Point::convert_to_frame(Point ref_pt) {

    Point new_pt = Point();

    // Shirt the car reference angle to 0 degrees.
    double shift_x = this->x - ref_pt.x;
    double shift_y = this->y - ref_pt.y;

    new_pt.x = (shift_x * cos(0 - ref_pt.yaw) - shift_y * sin(0 - ref_pt.yaw));
    new_pt.y = (shift_x * sin(0 - ref_pt.yaw) + shift_y * cos(0 - ref_pt.yaw));
    new_pt.yaw = normalize_rad(this->yaw - ref_pt.yaw);

    return new_pt;

  }

  Point Point::convert_from_frame(Point ref_pt) {
// Rotate back to map coordinates.
    Point new_pt = Point();

    new_pt.x = (ref_pt.x * cos(this->yaw) - ref_pt.y * sin(this->yaw));
    new_pt.y = (ref_pt.x * sin(this->yaw) + ref_pt.y * cos(this->yaw));
    new_pt.yaw = normalize_rad(this->yaw + ref_pt.yaw);

    new_pt.x += this->x;
    new_pt.y += this->y;

    return new_pt;
  }

  Point::Point() {
    this->x = 0;
    this->y = 0;
    this->yaw = 0;
  }

  Point::Point(double x, double y) {
    this->x = x;
    this->y = y;
    this->yaw = 0;
  }

  Point::Point(double x, double y, double yaw) {
    this->x = x;
    this->y = y;
    if (yaw < 0) {
      throw "Yaw is less than zero";
    }
    if (yaw > (2 * M_PI)) {
      throw "Yaw is greater than 2xPI";
    }
    this->yaw = yaw;
  }

  Point Point::clone() {
    return {this->x, this->y, this->yaw};

  }

  Spline::Spline(vector<Point> way_pts) {

    vector<double> x_pts;
    vector<double> y_pts;

    for (auto &way_pt : way_pts) {
      x_pts.push_back(way_pt.x);
      y_pts.push_back(way_pt.y);
    }

    // Create a spline.
    tk::spline spline;

    // Set x,y coordinates as anchor points of the spline.
    spline.set_points(x_pts, y_pts, true);

    this->spline = spline;
  }

  Point Spline::interpolate(double x) {
    Point new_pt = Point();
    new_pt.x = x;
    new_pt.y = this->spline(x);

    return new_pt;
  }

  Map::Map(string map_file, double max_s) {

    // The max s value before wrapping around the track back to 0
    this->max_s = max_s;

    // Waypoint map to read from
    ifstream in_map_(map_file.c_str(), ifstream::in);

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
      this->map_waypoints_x.push_back(x);
      this->map_waypoints_y.push_back(y);
      this->map_waypoints_s.push_back(s);
      this->map_waypoints_dx.push_back(d_x);
      this->map_waypoints_dy.push_back(d_y);
    }
  }
}
