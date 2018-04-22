#ifndef TRAJ_H
#define TRAJ_H

#include <cstdio>
#include <cassert>
#include <vector>
#include <cmath>
#include <algorithm>


using namespace std;

class Traj
{
	// This class describes a unidimension trajectory in s or d
public:
	Traj() {};
	Traj(vector<double> coeffs, double T_f); // builds a traj using coefficients and final time T_f
	~Traj();
	void setCost(double C); // set cost for Traj
	vector<double> getCoeffs(); // returns coefficinets of trajectory
	double getCost(); // returns cost
	double getDis(double t); // returns position at time t
	double getVel(double t); // returns velocity at time t
	double getAcc(double t); // returns acceleration at time t
	double getJer(double t); // returns Jerk at time t
	double getJerAvg(double t1, double t2); // returns average Jerk between time t1 and t2

	void operator = ( const Traj &Tr ); // initialize a Traj as equal to another Traj

	double a0;
	double a1;
	double a2;
	double a3;
	double a4;
	double a5;
	double T;
	double max_d;
	double max_Dd;
	double max_v;
	double max_a;
	double max_J;
	double avg_J;
	double Cost;
	bool CostIsSet;
	bool isVelCost;

private:
	void findMax (); // find max of position, velocity, acceleration and jerk
	
};

class combiTraj
{
	// This class describes a bi dimensional trajectory created combining two unidimensional trajectories
public:
	combiTraj() {};
	combiTraj ( Traj T1s, Traj T1d, double Tf ) ; // combines two Traj
	~combiTraj(){};

	bool collision(vector<double> params, double Tf); // checks for collisions, return true or false
	int dynamic(double vel_limit, double acc_limit, double Jer_limit); // checks for dynamic limits trespass (1 velocity, 2 acceleration, 3 Jerk)

	Traj Trs;
	Traj Trd;
	double T;

	double max_v;
	double max_a;
	double max_J;
	double Cost;


private:
	void findMax();  // find max of position, velocity, acceleration and jerk
};

#endif