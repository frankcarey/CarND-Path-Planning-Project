#ifndef TRAJ_H
#define TRAJ_H

#include <cstdio>
#include <cassert>
#include <vector>
#include <cmath>
#include <algorithm>


using namespace std;

class Trajectory
{
	// This class describes a unidimension trajectory in s or d
public:
	Trajectory() {};
	Trajectory(vector<double> coeffs, double T_f); // builds a traj using coefficients and final time T_f
	~Trajectory();
	double getDis(double t); // returns position at time t
	double getVel(double t); // returns velocity at time t
	double getAcc(double t); // returns acceleration at time t
	double getJerk(double t); // returns Jerk at time t
	double getAvgJerk2(double t1, double t2); // returns average Jerk between time t1 and t2

	void operator = ( const Trajectory &Tr ); // initialize a Trajectory as equal to another Trajectory

	double a0;
	double a1;
	double a2;
	double a3;
	double a4;
	double a5;
	double T;

  double getAvgJerk(double t);

};

class Trajectory2D {
public:
	Trajectory2D() {};
	Trajectory2D ( Trajectory sTraj, Trajectory dTraj, double time_final ) ;
	~Trajectory2D(){};

	int verifyLimits(double v_limit, double a_limit, double j_limit);
	Trajectory sTraj;
	Trajectory dTraj;
	double T;
	double Cost;
};

#endif