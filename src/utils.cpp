#include "utils.h"
#include <sstream>
#include <fstream>
#include "smoother.h"


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

  Position::Position(double x, double y): Position() {
    this->x = x;
    this->y = y;
  }

  Position::Position(double x, double y, double yaw): Position(x, y) {
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

  int Map::ClosestWaypoint(Position pt, vector<double> wp_x, vector<double> wp_y) {

    double closestLen = 100000; //large number
    int closestWaypoint = -1;

    for (int i = 0; i < wp_x.size(); i++) {
      double map_x = wp_x[i];
      double map_y = wp_y[i];
      double dist = utils::distance(pt.x, pt.y, map_x, map_y);
      if (dist < closestLen) {
        closestLen = dist;
        closestWaypoint = i;
      }
    }
    return closestWaypoint;
  }

  int Map::NextWaypoint(Position pt,  vector<double> wp_x, vector<double> wp_y ) {

    int closestWaypoint = ClosestWaypoint(pt, wp_x, wp_y);
    if (closestWaypoint == -1) {
      return -1;
    }

    double waypt_x = wp_x[closestWaypoint];
    double waypt_y = wp_y[closestWaypoint];

    double heading = atan2((waypt_y - pt.y), (waypt_x - pt.x));

    double angle = fabs(pt.yaw - heading);
    angle = min(2 * utils::pi() - angle, angle);

    if (angle > utils::pi() / 2) {
      closestWaypoint++;
      if (closestWaypoint == wp_x.size()) {
        closestWaypoint = 0;
      }
    }

    return closestWaypoint;
  }

  // Transform from Cartesian x,y coordinates to FrenetPos s,d coordinates
  FrenetPos Map::getFrenet(Position pt) {

    vector<double>* wp_x;
    vector<double>* wp_y;
    vector<double>* wp_s;
    int closest_wp = this->ClosestWaypoint(pt, interpolated_waypoints_x, interpolated_waypoints_y);

    if (closest_wp != -1) {
      wp_s = &interpolated_waypoints_s;
      wp_x = &interpolated_waypoints_x;
      wp_y = &interpolated_waypoints_y;
    } else {
      wp_s = &waypoints_s;
      wp_x = &waypoints_x;
      wp_y = &waypoints_y;
    }

    closest_wp = this->ClosestWaypoint(pt,(*wp_x), (*wp_y));
    int next_wp = this->NextWaypoint(pt, (*wp_x), (*wp_y));

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
      prev_wp = (int) (*wp_x).size() - 1;
    }

    double wp_x_delta = (*wp_x)[next_wp] - (*wp_x)[prev_wp];
    double wp_y_delta = (*wp_y)[next_wp] - (*wp_y)[prev_wp];
    double pt_wp_x_delta = pt.x - (*wp_x)[prev_wp];
    double pt_wp_y_delta = pt.y - (*wp_y)[prev_wp];

    // find the projection of x onto n
    double proj_norm = (pt_wp_x_delta * wp_x_delta + pt_wp_y_delta * wp_y_delta) /
        (wp_x_delta * wp_x_delta + wp_y_delta *wp_y_delta);
    double proj_x = proj_norm * wp_x_delta;
    double proj_y = proj_norm * wp_y_delta;

    double frenet_d = utils::distance(pt_wp_x_delta, pt_wp_y_delta, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - (*wp_x)[prev_wp];
    double center_y = 2000 - (*wp_y)[prev_wp];
    double centerToPos = distance(center_x, center_y, pt_wp_x_delta, pt_wp_y_delta);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef) {
      // todo: this seems broken!
      //frenet_d *= -1;
      cout << "negative frenet_d??";
    }

    // calculate s value by adding the s projection to the previous waypoint's s.
    double frenet_s = (*wp_s)[prev_wp] + distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};

  }

// Transform from FrenetPos s,d coordinates to Cartesian x,y
  Position Map::getXY(FrenetPos frenet) {
    //x = spline_x_s(s) + d * spline_dx_s(s)
    //y = spline_y_s(s) + d * spline_dy_s(s)

    int prev_wp = -1;

    vector<double>* wp_x;
    vector<double>* wp_y;
    vector<double>* wp_s;

    if (interpolated_waypoints_s.size() > 0 && frenet.s > interpolated_waypoints_s[0] && frenet.s < interpolated_waypoints_y.back()) {
      wp_s = &interpolated_waypoints_s;
      wp_x = &interpolated_waypoints_x;
      wp_y = &interpolated_waypoints_y;
    } else {
      wp_s = &waypoints_s;
      wp_x = &waypoints_x;
      wp_y = &waypoints_y;
    }

    while (frenet.s > (*wp_s)[prev_wp + 1] && (prev_wp < (int) (wp_s->size() - 1))) {
      prev_wp++;
    }

    auto wp2 = (prev_wp + 1) % wp_x->size();

    double heading = atan2(((*wp_y)[wp2] - (*wp_y)[prev_wp]),
                           ((*wp_x)[wp2] - (*wp_x)[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (frenet.s - (*wp_s)[prev_wp]);

    double seg_x = (*wp_x)[prev_wp] + seg_s * cos(heading);
    double seg_y = (*wp_y)[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + frenet.d * cos(perp_heading);
    double y = seg_y + frenet.d * sin(perp_heading);

    return {x, y, normalize_rad(heading)};

  }

//  Spline Map::getLocalSpline(Position pt, int n_points) {
//
//    if ((n_points % 1) == 0) {
//      // TODO: Throw exception unless is odd.
//    }
//    Spline spline{};
//
//    //waypoints
//
//    int pts_per_direction = n_points / 2;
//
//    int closest_idx = this->ClosestWaypoint(pt);
//
//    for(int i=pts_per_direction; i > 0; i--) {
//
//    }
//    return spline;
//  }

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

//  void Map::todo(double dist_inc) {
//    auto num_waypoints = this->waypoints_x.size();
//    int next_waypoint_index = this->NextWaypoint(pt , this->waypoints_x, this->waypoints_y);
//    vector<double> coarse_waypoints_s, coarse_waypoints_x, coarse_waypoints_y,
//        coarse_waypoints_dx, coarse_waypoints_dy;
//    for (int i = -wp_behind; i < wp_ahead; i++) {
//      // for smooting, take so many previous and so many subsequent waypoints
//      auto idx = (next_waypoint_index + i) % num_waypoints;
//
//      // correct for wrap in s for spline interpolation (must be continuous)
//      double current_s = this->waypoints_s[idx];
//      double base_s = this->waypoints_s[next_waypoint_index];
//      if (i < 0 && current_s > base_s) {
//        current_s -= this->max_s;
//      }
//      if (i > 0 && current_s < base_s) {
//        current_s += this->max_s;
//      }
//      coarse_waypoints_s.push_back(current_s);
//      coarse_waypoints_x.push_back(this->waypoints_x[idx]);
//      coarse_waypoints_y.push_back(this->waypoints_y[idx]);
//      coarse_waypoints_dx.push_back(this->waypoints_dx[idx]);
//      coarse_waypoints_dy.push_back(this->waypoints_dy[idx]);
//    }
//
//
//    // interpolation parameters
//    double dist_inc = 0.5;
//    auto num_interpolation_points = (int) ((coarse_waypoints_s[coarse_waypoints_s.size() - 1] - coarse_waypoints_s[0]) / dist_inc);
//    vector<double> interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y,
//        interpolated_waypoints_dx, interpolated_waypoints_dy;
//    // interpolated s is simply...
//    interpolated_waypoints_s.push_back(coarse_waypoints_s[0]);
//    for (int i = 1; i < num_interpolation_points; i++) {
//      this->interpolated_waypoints_s.push_back(coarse_waypoints_s[0] + i * dist_inc);
//    }
//    this->interpolated_waypoints_x = interpolate_points(coarse_waypoints_s, coarse_waypoints_x, dist_inc,
//                                                        num_interpolation_points);
//    this->interpolated_waypoints_y = interpolate_points(coarse_waypoints_s, coarse_waypoints_y, dist_inc,
//                                                        num_interpolation_points);
//    this->interpolated_waypoints_dx = interpolate_points(coarse_waypoints_s, coarse_waypoints_dx, dist_inc,
//                                                         num_interpolation_points);
//    this->interpolated_waypoints_dy = interpolate_points(coarse_waypoints_s, coarse_waypoints_dy, dist_inc,
//                                                         num_interpolation_points);
//
//  }

  void Map::update_local_waypoints(Position pt, int wp_behind, int wp_ahead){
    auto num_waypoints = this->waypoints_x.size();
    int next_waypoint_index = this->NextWaypoint(pt , this->waypoints_x, this->waypoints_y);
    vector<double> coarse_waypoints_s, coarse_waypoints_x, coarse_waypoints_y,
        coarse_waypoints_dx, coarse_waypoints_dy;
    for (int i = -wp_behind; i < wp_ahead; i++) {
      // for smooting, take so many previous and so many subsequent waypoints
      auto idx = (next_waypoint_index + i) % num_waypoints;

      // correct for wrap in s for spline interpolation (must be continuous)
      double current_s = this->waypoints_s[idx];
      double base_s = this->waypoints_s[next_waypoint_index];
      if (i < 0 && current_s > base_s) {
        current_s -= this->max_s;
      }
      if (i > 0 && current_s < base_s) {
        current_s += this->max_s;
      }
      coarse_waypoints_s.push_back(current_s);
      coarse_waypoints_x.push_back(this->waypoints_x[idx]);
      coarse_waypoints_y.push_back(this->waypoints_y[idx]);
      coarse_waypoints_dx.push_back(this->waypoints_dx[idx]);
      coarse_waypoints_dy.push_back(this->waypoints_dy[idx]);
    }


    // interpolation parameters
    double dist_inc = 0.5;
    auto num_interpolation_points = (int) ((coarse_waypoints_s[coarse_waypoints_s.size() - 1] - coarse_waypoints_s[0]) / dist_inc);
    cout << "num_interpolation_points: " << num_interpolation_points << "\n";
    vector<double> interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y,
        interpolated_waypoints_dx, interpolated_waypoints_dy;
    // interpolated s is simply...
    for (int i = 0; i < num_interpolation_points; i++) {
      interpolated_waypoints_s.push_back(coarse_waypoints_s[0] + i * dist_inc);
    }

    this->interpolated_waypoints_s = interpolated_waypoints_s;

    this->interpolated_waypoints_x = interpolate_points(coarse_waypoints_s, coarse_waypoints_x, dist_inc,
                                                  num_interpolation_points);
    this->interpolated_waypoints_y = interpolate_points(coarse_waypoints_s, coarse_waypoints_y, dist_inc,
                                                  num_interpolation_points);
    this->interpolated_waypoints_dx = interpolate_points(coarse_waypoints_s, coarse_waypoints_dx, dist_inc,
                                                   num_interpolation_points);
    this->interpolated_waypoints_dy = interpolate_points(coarse_waypoints_s, coarse_waypoints_dy, dist_inc,
                                                   num_interpolation_points);

  }

  FrenetPos::FrenetPos() : s(0), d(0) {}
  FrenetPos::FrenetPos(double s, double d) : s(s), d(d) {}


}

