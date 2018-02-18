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

constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);
string hasData(string s);
double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
void generate_spline_path(float current_s, float current_d, float target_d, float yaw, float velocity, float acceleration,
                          vector<float> &previous_path_x, vector<float> &previous_path_y,
                          vector<float> &next_x_vals, vector<float> &next_y_vals,
                          vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y);


#endif //UTILS_H

