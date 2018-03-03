#include "utils.h"
#include <sstream>
#include <fstream>


using namespace std;

namespace utils {

  double deg2rad(double x) { return x * pi() / 180; }

  double rad2deg(double x) { return x * 180 / pi(); }


  double distance(double x, double y) {
    return sqrt(x*x + y*y);
  }

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

  vector<Position> pt_from_xy_list(XYList l) {
    assert(!l.x_list.empty());
    assert(l.x_list.size() == l.y_list.size());

    vector<Position> points;

    for (int i = 0; i < l.x_list.size(); i++) {
      Position pt = Position(l.x_list[i], l.y_list[i]);
      points.push_back(pt);
    }
    return points;
  }

  XYList xy_from_pt_list(vector<Position> pts) {
    assert(!pts.empty());

    vector<double> x_list;
    vector<double> y_list;

    for (auto &pt : pts) {
      x_list.push_back(pt.x);
      x_list.push_back(pt.y);
    }
    return {x_list, y_list};
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

  tk::spline create_spline(vector<Position> way_pts) {

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

  Position Position::convert_to_frame(Position ref_pt) {

    Position new_pt = Position();

    // Shirt the car reference angle to 0 degrees.
    double shift_x = this->x - ref_pt.x;
    double shift_y = this->y - ref_pt.y;

    new_pt.x = (shift_x * cos(0 - ref_pt.yaw) - shift_y * sin(0 - ref_pt.yaw));
    new_pt.y = (shift_x * sin(0 - ref_pt.yaw) + shift_y * cos(0 - ref_pt.yaw));
    new_pt.yaw = normalize_rad(this->yaw - ref_pt.yaw);

    return new_pt;

  }

  Position Position::convert_from_frame(Position ref_pt) {
// Rotate back to map coordinates.
    Position new_pt = Position();

    new_pt.x = (ref_pt.x * cos(this->yaw) - ref_pt.y * sin(this->yaw));
    new_pt.y = (ref_pt.x * sin(this->yaw) + ref_pt.y * cos(this->yaw));
    new_pt.yaw = normalize_rad(this->yaw + ref_pt.yaw);

    new_pt.x += this->x;
    new_pt.y += this->y;

    return new_pt;
  }

  Position::Position() {
    this->x = 0;
    this->y = 0;
    this->yaw = 0;
  }

  Position::Position(double x, double y) {
    this->x = x;
    this->y = y;
    this->yaw = 0;
  }

  Position::Position(double x, double y, double yaw) {
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

  Position Position::clone() {
    return {this->x, this->y, this->yaw};

  }


  Spline::Spline(vector<Position> way_pts) {

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

  Position Spline::interpolate(double x) {
    Position new_pt = Position();
    new_pt.x = x;
    new_pt.y = this->spline(x);

    return new_pt;
  }

  Map::Map(string map_file, double max_s, double speed_limit_mph, double n_lanes) {

    // The max s value before wrapping around the track back to 0
    this->max_s = max_s;
    this->speed_limit_mph = speed_limit_mph;
    this->n_lanes = n_lanes;

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
      this->waypoints_x.push_back(x);
      this->waypoints_y.push_back(y);
      this->waypoints_s.push_back(s);
      this->waypoints_dx.push_back(d_x);
      this->waypoints_dy.push_back(d_y);
    }
  }

  double Map::speed_limit() {
    // Convert miles per hour to meters per second.
    return utils::from_mph(this->speed_limit_mph);
  }

  bool Map::is_lane_available(int lane) {
    return (lane >=0) && (lane < this->n_lanes);
  }

  int Map::ClosestWaypoint(Position pt) {

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < this->waypoints_x.size(); i++) {
      double map_x = this->waypoints_x[i];
      double map_y = this->waypoints_y[i];
      double dist = utils::distance(pt.x, pt.y, map_x, map_y);
      if (dist < closestLen) {
        closestLen = dist;
        closestWaypoint = i;
      }
    }
    return closestWaypoint;
  }

  int Map::NextWaypoint(Position pt) {

    int closestWaypoint = ClosestWaypoint(pt);

    double waypt_x = this->waypoints_x[closestWaypoint];
    double waypt_y = this->waypoints_y[closestWaypoint];

    double heading = atan2((waypt_y - pt.y), (waypt_x - pt.x));

    double angle = fabs(pt.yaw - heading);
    angle = min(2 * utils::pi() - angle, angle);

    if (angle > utils::pi() / 4) {
      closestWaypoint++;
      if (closestWaypoint == waypoints_x.size()) {
        closestWaypoint = 0;
      }
    }

    return closestWaypoint;
  }

  // Transform from Cartesian x,y coordinates to FrenetPos s,d coordinates
  FrenetPos Map::getFrenet(Position pt) {
    int next_wp = this->NextWaypoint(pt);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
      prev_wp = (int) waypoints_x.size() - 1;
    }

    double n_x = waypoints_x[next_wp] - waypoints_x[prev_wp];
    double n_y = waypoints_y[next_wp] - waypoints_y[prev_wp];
    double x_x = pt.x - waypoints_x[prev_wp];
    double x_y = pt.y - waypoints_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = utils::distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - waypoints_x[prev_wp];
    double center_y = 2000 - waypoints_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
      frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; i++) {
      frenet_s += distance(waypoints_x[i], waypoints_y[i], waypoints_x[i + 1], waypoints_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};

  }

// Transform from FrenetPos s,d coordinates to Cartesian x,y
  Position Map::getXY(FrenetPos frenet) {

    int prev_wp = -1;

    while (frenet.s > waypoints_s[prev_wp + 1] && (prev_wp < (int) (waypoints_s.size() - 1))) {
      prev_wp++;
    }

    auto wp2 = (prev_wp + 1) % waypoints_x.size();

    double heading = atan2((waypoints_y[wp2] - waypoints_y[prev_wp]), (waypoints_x[wp2] - waypoints_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (frenet.s - waypoints_s[prev_wp]);

    double seg_x = waypoints_x[prev_wp] + seg_s * cos(heading);
    double seg_y = waypoints_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + frenet.d * cos(perp_heading);
    double y = seg_y + frenet.d * sin(perp_heading);

    return {x, y, normalize_rad(heading)};

  }

  int Map::getXYLane(Position pos) {
    return this->getFrenetLane(this->getFrenet(pos));
  }

  int Map::getFrenetLane(FrenetPos frenet) {
    // assumes 0 is the left most lane.
    return (int) round(frenet.d / 4 - 0.5);
  }

  Position Map::position_at(Position pos, double timedelta) {
    FrenetPos fpos = this->getFrenet(pos);
    // TODO: Convert to new position.
    return this->getXY(fpos);
  }

  FrenetPos::FrenetPos() : s(0), d(0) {}
  FrenetPos::FrenetPos(double s, double d) : s(s), d(d) {}


}
