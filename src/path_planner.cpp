//
// Created by Carey, Frank, Sr. on 3/3/18.
//

#include "path_planner.h"

using namespace vehicle;

PathPlanner::PathPlanner(){
}

double PathPlanner::calculate_cost(double speed_limit, Trajectory2D &trajectory) {
  double cost = 0;
  // Add s-velocity delta cost.
  cost += (1000.) * abs(trajectory.sTraj.getVel(trajectory.T) - speed_limit);
  // Add average s-jerk cost.
  cost += (10.) * trajectory.sTraj.getAvgJerk(trajectory.T);
  // Add average d-jerk cost.
  cost += (10.) * trajectory.dTraj.getAvgJerk(trajectory.T);
  // Add cost for changing lanes.
  cost += (1.) * abs(trajectory.dTraj.getDis(0) - trajectory.dTraj.getDis(trajectory.T));

  return cost;
}


int PathPlanner::minimal_cost_trajectory(vector<Trajectory2D> combSet) {
  // find minimal cost trajectory
  double min_Comb_Cost = 10e10;
  int min_Comb_idx = 0;
  for (int k = 0; k < combSet.size(); k++) {
    if (min_Comb_Cost > combSet[k].Cost) {
      min_Comb_Cost = combSet[k].Cost;
      min_Comb_idx = k;
    }
  }
  return min_Comb_idx;
}


// generate set of trajectories s(t) and d(t)
vector<Trajectory2D> PathPlanner::generate_trajectories(VehicleController carCtl, vector<double> conds_s, vector<double> conds_d,
                                                          double time_horizon, vector< vector<double> > & near_cars) {
  vector<Trajectory> sSet; // return set
  vector<Trajectory> dSet; // return set
  vector<Trajectory2D> combiSet;

  // Generate candidate s trajectories.
  for (int speed_inc=0 ; speed_inc < 7 ; speed_inc++)
  {
    for (int time_inc=0; time_inc < 6 ; time_inc++)
    {
      // Clone the s vector and update the speed.
      vector<double> s_conds_temp(conds_s);
      s_conds_temp[3] = carCtl.speed_limit - (carCtl.speed_limit - carCtl.speed_minimum)*speed_inc/6.;

      double time_step_size = (time_horizon - 1.5)/5.;

      // Ensure time is in the future.
      double final_time = time_horizon-time_step_size*time_inc;
      if (final_time > 0) {
        Trajectory s_traj = Trajectory(sPoly(s_conds_temp, final_time), final_time);
        sSet.push_back(s_traj);
      }
    }
  }

  // Generate candidate d trajectories.
  vector<double> lanes = {2., 6., 10.};
  for (int lane_idx=0 ; lane_idx < 3 ; lane_idx++)
  {
    for (int time_inc=0; time_inc < 6 ; time_inc++)
    {
      vector<double> d_conds_temp(conds_d);
      d_conds_temp[3] = lanes[lane_idx];

      double time_step_size = (time_horizon - 2)/6.;

      // Ensure time is in the future.
      double final_time = time_horizon - time_step_size*time_inc - 1;
      if (final_time > 0)
      {
        Trajectory d_traj = Trajectory(dPoly(d_conds_temp, final_time), final_time);
        dSet.push_back(d_traj);
      }
    }
  }

  vector<Trajectory2D> combSet; // return set

  // Combine Trajectories that don't lead to collisions.
  for (int i = 0; i<sSet.size(); i++) {
    for (int j = 0; j<dSet.size(); j++) {
      Trajectory2D comb_traj = Trajectory2D(sSet[i], dSet[j], time_horizon); // combine 2 trajectories into 1 Trajectory2D
      int wrong_dyn = comb_traj.verifyLimits(carCtl.speed_limit, carCtl.acc_limit, carCtl.jerk_limit); // check for dynamic limit trespass

      if ( wrong_dyn == 0 ) {
        // if Trajectory2D survives dynamic check, check for collisions
        bool coll = false;
        int idx = 0;

        while ( (idx < near_cars.size()) && (!coll) ) {
          // for every near car get s,d and velocity
          double nc_s = near_cars[idx][5];
          double nc_d = near_cars[idx][6];
          double nc_vx = near_cars[idx][3];
          double nc_vy = near_cars[idx][4];
          double nc_v = sqrt(nc_vx*nc_vx + nc_vy*nc_vy);

          coll = (carCtl.trackMap->collision_test(comb_traj, {nc_s,nc_d,nc_v}, time_horizon));
          idx++;
        }
        if (!coll) {
          comb_traj.Cost = calculate_cost(carCtl.speed_limit, comb_traj);
          combSet.push_back(comb_traj); // if Trajectory2D survives the collisions check, add to the eligible trajectories set
        }
      }
    }
  }

  return combSet;
}
/**
 * Create an optimal polynomial wrt s dimension.
 * @param conds
 * @param T
 * @return
 */
vector<double> PathPlanner::sPoly(vector<double> conds, double T) {

  double s_start = conds[0];
  double v_start = conds[1];
  double a_start = conds[2];
  double v_final = conds[3];
  double a_final = conds[4];

  double v_delta = v_final - (v_start + a_start*T);
  double a_delta = a_final-a_start;

  vector<double> coeffs = {
      s_start,
      v_start,
      a_start*0.5,
      (3.*v_delta - a_delta*T)/(3.*T*T),
      (a_delta*T - 2.*v_delta)/(4.*T*T*T)
  };
  return coeffs;
}
/**
 * Create an optimal polynomial wrt d dimension.
 * @param conds
 * @param T
 * @return
 */
vector<double> PathPlanner::dPoly(vector<double> conds, double T)
{
  double d_start = conds[0];
  double v_start = conds[1];
  double a_start = conds[2];
  double d_final = conds[3];
  double v_final = conds[4];
  double a_final = conds[5];

  double d_delta = (d_final - d_start - v_start*T - 0.5*a_start*T*T);
  double v_delta = (v_final - v_start - a_start*T);
  double a_delta = a_final - a_start;

  vector<double> coeffs = {
      d_start,
      v_start,
      a_start*0.5,
      (20.*d_delta - 8.*v_delta*T + a_delta*T*T)/(2.*T*T*T),
      (-15.*d_delta + 7.*v_delta*T - a_delta*T*T)/(T*T*T*T),
      (12.*d_delta - 6.*v_delta*T + a_delta*T*T)/(2.*T*T*T*T*T),
  };
  return coeffs;
}


