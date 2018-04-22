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

  double calcPoly (std::vector<double> coeffs, double t)
  {
    // This functions calculates the value of a polynomial at time t and returns its value

    //INPUT:
    // a vector of all coefficients for the polynomial sorted from lowest degree to highest
    // the time t at which evaluate the polynomial

    //OUTPUT:
    // the value of the polynomial at time t

    double pol = 0.;
    for (int i = 0; i < coeffs.size(); i++)
    {
      pol += coeffs[i] * pow(t, i);
    }
    return pol;
  }


  std::vector<double> parabolicInterpol(std::vector<double> X, std::vector<double> Y, int center, double ds, double d)
  {
    // This functions interpolates a 2nd grade polynomial between 3 waypoints X,Y and then uses ds and d to estimate the x,y position
    // of a point between the center waypoint and the next

    //INPUT:
    // vector of X coordinates of 3 waypoints
    // vector of Y coordinates of 3 waypoints
    // index of the central waypoint
    // arc lenght along the parabola ds
    // coordinate d

    //OUTPUT:
    // a vector of (x,y) coordinate for point (ds,d)



    // transform to reference system of center point
    double x0 = X[center-1] - X[center];
    double x1 = X[center] - X[center];
    double x2 = X[center+1] - X[center];

    double y0 = Y[center-1] - Y[center];
    double y1 = Y[center] - Y[center];
    double y2 = Y[center+1] - Y[center];

    double den_X = (x0-x1)*(x0-x2)*(x1-x2);
    double den_y = (y0-y1)*(y0-y2)*(y1-y2);
    double disc_x = (x0-x1)*(x1-x2);
    double disc_y = (y0-y1)*(y1-y2);
    bool rotate = false;

    if (disc_x <= 0 )
    {
      //rotate reference system, so that (x,y) -> (-y,x)

      double tx0 = -y0;
      double tx1 = -y1;
      double tx2 = -y2;

      y0 = x0;
      y1 = x1;
      y2 = x2;

      x0 = tx0;
      x1 = tx1;
      x2 = tx2;

      std::vector<double> TX;

      for (int i =0;i<X.size();i++)
      {
        TX.push_back(-Y[i]);
        Y[i]=X[i];
        X[i]=TX[i];
      }

      rotate = true;

    }

    // Calculate 3 parameters of the parabola passing by the 3 waypoints y=ax^2+bx+c
    double den = (x0-x1)*(x0-x2)*(x1-x2);
    double a = ( x2*(y1-y0) + x1*(y0-y2) + x0*(y2-y1) )/den;
    double b = ( x2*x2*(y0-y1) + x1*x1*(y2-y0) +x0*x0*(y1-y2) )/den;
    double c = ( x1*x2*(x1-x2)*y0 + x2*x0*(x2-x0)*y1 +x0*x1*(x0-x1)*y2 )/den;


    double sum = 0.;
    int idx = 0;

    double X1 = X[center]-X[center]; // transform to reference system of center point
    double X2 = X[center+abs(ds)/ds]-X[center]; // second integration limit is the previous or successive point of center, according to ds sign

    double h = (X2-X1)/50.;

    // the arc lenght of a parabola is the definite integral of sqrt(1+f'(x)^2) in dx
    double u1 = 2.*a*X1 +b; // helper variable
    double g1 = u1*sqrt(1+u1*u1) + log(abs(sqrt(1+u1*u1) + u1)); // primitive of sqrt(1+f'(x)^2) calculated in X1

    double xe2=X1;

    // EVALUATE xe2 at which the arc lenght equals |ds| with 1e-11 tolerance or with 10000000 max iterations, whatever happens first
    while (( abs(abs(ds) - sum) > 1e-11) && (idx < 10000000))
    {

      xe2 += h;
      double u2 = 2.*a*xe2 + b;
      double g2 = (u2*sqrt(1+u2*u2) + log(abs(sqrt(1+u2*u2) + u2))); // primitive of sqrt(1+f'(x)^2) calculated in xe2

      sum = abs((g2 - g1)/(4.*a)); // arc lenght from X1 to xe2
      if (sum > abs(ds) ) // if arc lenght is greater than |ds| go back one step and divide h by 2
      {
        xe2 -= h;
        h = h/2.;
      }
      idx++;
    }
    double xp = xe2;
    double yp = calcPoly({c,b,a},xp);
    double heading = atan2(2.*a*xp + b, 1.); //calculate heading of parabola at point (xp, yp=2axp+b)

    // transform back to global reference system
    xp += X[center];
    yp += Y[center];

    if (rotate)
    {
      //rotate back
      double txp= xp;
      xp = yp;
      yp = -txp;


      if (x1-x0 > 0.)
      {
        heading = heading + M_PI;
      }

      // add d offset using heading

      xp += d * cos(heading);
      yp += d * sin(heading);


    } else {


      if (x1-x0 < 0.)
      {
        heading = heading + M_PI;
      }
      heading = heading-M_PI/2.;
      xp += d * cos(heading);
      yp += d * sin(heading);
    }

    return{xp,yp};
  }


  vector<double> parabolicGetXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
  {
    // This functions transform from s,d coordinates to global x,y coordinates using a waypoint maps of the highway
    // Instead of a linear interpolation, it uses two parabolic interpolation and then calculates a weighted mean.
    // The first interpolation is made using the previous waypoint and the immidiately successive and previous waypoint,
    // the second interpolation is made using the previous waypoint and the immidiately 2 successive waypoints.
    // Then a weighted mean of the two points is calculated using, as weights, the inverse of the squared distance from the
    // previous waypoint and the next waypoint

    //INPUT:
    // s coordinate
    // d coordinate
    // s values of waypoints
    // x values of waypoints
    // y values of waypoints

    //OUTPUT:
    // a vector of (x,y) coordinate for point (s,d)

    double max_s = 6945.554; //max s value for waypoints
    while (s > max_s)
    {
      s -= max_s;
    }

    int prev_wp = -1;

    // find the previous waypoint

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
      prev_wp++;
    }


    int cubic_num; //helper index of the previous waypoint

    vector<double> X; // X coordinates of nearest waypoints used for interpolation
    vector<double> Y; // Y coordinates of nearest waypoints used for interpolation

    // fill X and Y with 1 previous waypoint and 2 successive waypoints,
    // if previous waypoint is 0 then start from the last waypoint in the map

    if (prev_wp >=1)
    {
      cubic_num = 1;
      for (int i = -1; i < 3; i++)
      {
        X.push_back( maps_x[(prev_wp + i)%maps_x.size()] );
        Y.push_back( maps_y[(prev_wp + i)%maps_x.size()] );
      }
    }
    else
    {
      cubic_num = 1;
      for (int i = maps_x.size() -1 ; i < maps_x.size() + 3; i++)
      {
        X.push_back( maps_x[i%maps_x.size()] );
        Y.push_back( maps_y[i%maps_x.size()] );
      }
    }

    double ds_p = s - maps_s[prev_wp]; //distance in s from previous waypoint

    std::vector<double> XYp = parabolicInterpol(X,Y, cubic_num, ds_p, d); // calculate x,y using the previous waypoint as the central waypoint for interpolation

    double ds_s; // distance in s from the next waypoint
    if (prev_wp == maps_s.size() - 1 )
    {
      ds_s = s - max_s;
    }
    else
    {
      ds_s = s - maps_s[(prev_wp+1)];
    }


    std::vector<double> XYs = parabolicInterpol(X,Y, cubic_num+1, ds_s, d); // calculate x,y using the next waypoint as the central waypoint for interpolation

    // calculate the weighted mean of the two interpolations using the inverse sqaure of the distance from previous and next waypoint
    int n_exp=-2;
    double p1 = pow(ds_p,n_exp);
    double p2 = pow(ds_s,n_exp);
    double norm =p1+p2;
    double x = (XYp[0]*p1 + XYs[0]*p2)/(norm);
    double y = (XYp[1]*p1 + XYs[1]*p2)/(norm);

    return {x,y};

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

  FrenetPos::FrenetPos() : s(0), d(0) {}

  FrenetPos::FrenetPos(double s, double d) : s(s), d(d) {}

  FrenetPos FrenetPos::clone() {
    FrenetPos ret = {this->s, this->d};
    return ret;
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
    const bool USE_IMPROVED_GETXY  = true;
    if (USE_IMPROVED_GETXY) {
      vector<double> xy = parabolicGetXY(frenet.s, frenet.d, waypoints_s, waypoints_x, waypoints_y);
      return {xy[0], xy[1], 0};
    }

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
    //return (int) round((frenet.d - 2) / 4);
    // TODO: The above function should return a better result?
    return (int) floor(frenet.d/4);
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




}

