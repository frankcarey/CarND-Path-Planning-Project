#include "utils.h"
#include <fstream>
#include <cmath>

using namespace std;

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
    double dist = distance(x, y, map_x, map_y);
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
  angle = min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = (int) maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x.at(prev_wp);
  double n_y = maps_y[next_wp] - maps_y.at(prev_wp);
  double x_x = x - maps_x.at(prev_wp);
  double x_y = y - maps_y.at(prev_wp);

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x.at(prev_wp);
  double center_y = 2000 - maps_y.at(prev_wp);
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
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

void generate_spline_path(float current_s, float current_d, float target_d, float yaw, float target_speed,
                          vector<float> &previous_path_x, vector<float> &previous_path_y,
                           vector<float> &next_x_vals, vector<float> &next_y_vals,
                          vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y)
{

  auto prev_size = previous_path_x.size();

  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  // Later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
  vector<double> way_pts_x; // ptsx in video
  vector<double> way_pts_y; // ptsy in video

  // Reference x, y, and yaw states. Either we will reference the starting point as where the car is or
  // at the previous paths end point.
  vector<double> car_xy = getXY(current_s, current_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  double ref_x = car_xy[0];
  double ref_y = car_xy[1];
  double ref_yaw = deg2rad(yaw);

  // cout << ref_x << ":" << ref_y << ":" << ref_yaw << "\n";

  // If the previous points are almost empty, use the car as a starting reference
  // and infer two points based on the car's current position and heading.
  if (prev_size < 2) {
    // Use the two points that make the path tangent to the car.
    double prev_car_x = ref_x - cos(yaw);
    double prev_car_y = ref_y - sin(yaw);
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
  vector<double> next_wp0 = getXY(current_s + 60, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(current_s + 120, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(current_s + 180, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

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

    double N = (target_dist / (.02 * target_speed / 2.24));
    float &&x_point = (float) (x_add_on + (target_x) / N);
    float &&y_point = (float) spline(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Rotate back to map coordinates.
    x_point = (float) (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (float) (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}