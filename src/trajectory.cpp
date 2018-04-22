
#include <cstdio>
#include <cassert>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

#include "trajectory.h"

using namespace std;

Trajectory2D::Trajectory2D(Trajectory sTraj, Trajectory dTraj, double time_final)
{
  this->sTraj = sTraj;
  this->dTraj = dTraj;
  this->T = time_final;
}

int Trajectory2D::verifyLimits(double v_limit, double a_limit, double j_limit)
{
  double dt = 0.02;
  // Velocity limit
  for (int i = 0; i < T/dt; i++) {
    if (sqrt(pow(sTraj.getVel(i*dt),2.) + pow(dTraj.getVel(i*dt),2.)) > 1.05*v_limit) {
      return 1;
    }
  }

  double avga = 0;
  int numa=0;
  dt = 0.02;
  // Acceleration limit
  for (int i = 0; i < T/dt; i++) {
    avga+=(sqrt(pow(sTraj.getAcc(i*dt),2.) + pow(dTraj.getAcc(i*dt),2.)));
    numa++;
    if (numa==10) {
      if (avga/numa>a_limit) {
        return 2;
      }
      numa--;
      avga-= (sqrt(pow(sTraj.getAcc((i-numa)*dt),2.) + pow(dTraj.getAcc((i-numa)*dt),2.)));
    }
  }

  dt = 1;
  // Jerk limit
  for (int i = 0; i < T/dt -1 ; i++) {
    if ( sqrt( pow(sTraj.getAvgJerk2(i * dt, (i + 1) * dt),2.) +
               pow(dTraj.getAvgJerk2(i * dt, (i + 1) * dt),2.) ) > j_limit) {
      return 3;
    }
  }


  return 0;
}

Trajectory::Trajectory(std::vector<double> coeffs, double T_f)
{
  a0 = coeffs[0];
  a1 = coeffs[1];
  a2 = coeffs[2];
  a3 = coeffs[3];
  a4 = coeffs[4];
  if (coeffs.size()>5)
  {
    a5 = coeffs[5];
  } else {
    a5=0.;
  }

  T=T_f;

}

Trajectory::~Trajectory()
{

}

double Trajectory::getDis(double t)
{
  return (a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t);
}

double Trajectory::getVel(double t) {
  return (a1 + 2.*a2*t + 3.*a3*t*t + 4.*a4*t*t*t+ 5.*a5*t*t*t*t);
}

double Trajectory::getAcc(double t) {
  return (2.*a2 + 6.*a3*t + 12.*a4*t*t + 20.*a5*t*t*t);
}

double Trajectory::getJerk(double t) {
  return (6.*a3 + 24.*a4*t + 60.*a5*t*t);
}

double Trajectory::getAvgJerk2(double t1, double t2)
{
  if (t2==t1) {return 0.;}
  return sqrt(getAvgJerk(t2) - getAvgJerk(t1) / (t2-t1));
}

double Trajectory::getAvgJerk(double t) {
  return sqrt((3.*a3*a3*T + 12.*a3*a4*T*T + (16.*a4*a4 + 20.*a3*a5)*T*T*T +
              60.*a4*a5*T*T*T*T + 60.*a5*a5*T*T*T*T*T )/T);
}

void Trajectory::operator = (const Trajectory &Tr )
{
  a0 = Tr.a0;
  a1 = Tr.a1;
  a2 = Tr.a2;
  a3 = Tr.a3;
  a4 = Tr.a4;
  a5 = Tr.a5;
  T = Tr.T;
}

