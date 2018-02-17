#
//
// Created by Carey, Frank, Sr. on 2/17/18.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H


#include <vector>
#include <string>
#include <cmath>

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


#endif //PATH_PLANNING_UTILS_H

