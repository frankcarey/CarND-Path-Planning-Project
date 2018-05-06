//
// Created by Carey, Frank, Sr. on 3/3/18.
//

#include "path_planner.h"

using namespace vehicle;

PathPlanner::PathPlanner(){
}

std::pair<fsm::STATE, vector<Vehicle>> PathPlanner::choose_next_state(VehicleController &ctrl, map<int, vector<Vehicle>> &other_vehicle_predictions) {
  /*
  Here you can implement the transition_function code from the Behavior Planning Pseudocode
  classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
  to the next state.

  INPUT: A predictions map. This is a map of vehicle id keys with predicted
      vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
      the vehicle at the current timestep and one timestep in the future.
  OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

  */
  int curr_lane = ctrl.get_lane();
  vector<fsm::STATE> states = ctrl.fsm->successor_states(curr_lane);
  double cost;
  vector<double> costs;
  vector<fsm::STATE> final_states;
  vector<vector<Vehicle>> final_trajectories;

  for (fsm::STATE &state : states) {
    vector<Vehicle> candidate_trajectory = ctrl.generate_trajectory(state, other_vehicle_predictions);
    if (!candidate_trajectory.empty()) {
      cout << "state cost for: " << state << "\n";
      cost = this->calculate_cost(ctrl, candidate_trajectory, other_vehicle_predictions);
      costs.push_back(cost);
      final_trajectories.push_back(candidate_trajectory);
      final_states.push_back(state);
    }
  }

  auto best_cost = min_element(begin(costs), end(costs));
  auto best_idx = (int) distance(begin(costs), best_cost);

  return {final_states[best_idx], final_trajectories[best_idx]};
}


double PathPlanner::calculate_cost(VehicleController &ctrl, vector<Vehicle> &candidate_trajectory, std::map<int, vector<Vehicle>> &other_vehicle_predictions) {
  /*
  Sum weighted cost functions to get total cost for trajectory.
  */
  double cost = 0.0;

  cost += fabs(ctrl.trackMap->speed_limit() - candidate_trajectory.back().v());

  // Make it change lanes..
  int lane = ctrl.trackMap->getFrenetLane(candidate_trajectory.back().position());
  cout << "lane: " << lane << " \n";
  int car_ahead_id = ctrl.get_vehicle_ahead(lane, other_vehicle_predictions);
  if(car_ahead_id > 0) {
    Vehicle car_ahead = other_vehicle_predictions[car_ahead_id].back();
    cost += fabs(ctrl.trackMap->speed_limit() - car_ahead.v());
  }

  cout << "\n";
  cout << "cost: " << cost << "\n";
  cout << "\n";
  return cost;
}


int PathPlanner::minimal_cost_trajectory(vector<combiTraj> combSet) {
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
vector<combiTraj> PathPlanner::generate_trajectories(VehicleController carCtl, vector<double> conds_s, vector<double> conds_d,
                                                          double time_horizon, double l_desired, vector< vector<double> > & near_cars)
{

  // This function generates a full set of unidimensional trajectories for s and d using jerk minimizing polynomials
  // and different values of final boundary conditions.
  //
  // This function uses the class Traj that can be found in trajectory.h header file

  // INPUT:
  //  conds_s is the set of boundary conditions for s trajectories
  //  conds_d is the set of boundary conditions for d trajectories
  //  time_horizion is the final time for the end of trajectories
  //  s_goal is the desired speed for the car
  //  l_desired is the desired lane for the car
  //  limits is a vector of dynamic limits (velocity, acceleration and jerk)
  //  max_min is a vector to store useful variables for cost normalization

  // OUTPUT:
  // a vector of 2 sets of trajectories, the first is the s trajectory set and the second is the d trajectory set

  vector<double> conds;
  vector<double> d_conds;

  double lane_desired = l_desired;

  vector<Traj> longSet; // return set
  vector<Traj> lateSet; // return set

  // a set of useful variables for cosst normalization
  double max_ds = 0.;
  double min_ds = 10e10;
  double max_avgJ = 0.;
  double min_avgJ = 10e10;
  double max_T = 0.;
  double min_T = 10e10;

  double max_dd = 0.;
  double min_dd = 10e10;
  double d_max_avgJ = 0.;
  double d_min_avgJ = 10e10;
  double d_max_T = 0.;
  double d_min_T = 10e10;


  // create s trajectories using different time of manouver and final speed
  for (int i=0 ; i < 7 ; i++)
  {
    for (int j=0; j < 6 ; j++)
    {
      conds.push_back(conds_s[0]);
      conds.push_back(conds_s[1]);
      conds.push_back(conds_s[2]);
      conds.push_back(carCtl.speed_limit - (carCtl.speed_limit - carCtl.speed_minimum)*i/6. );
      conds.push_back(conds_s[4]);

      double Tj = time_horizon;
      double dtj = (Tj - 1.5)/5.;

      if (Tj-dtj*j > 0)
      {
        Traj s_traj = Traj(keepVelPoly(conds, Tj - j*dtj), Tj -j*dtj);
        conds.clear();

        longSet.push_back(s_traj);
        // useful variables for cost normalization
        double ds = abs(s_traj.getVel(s_traj.T) - carCtl.speed_limit);
        max_ds = max(max_ds,ds);
        min_ds = min(min_ds,ds);
        max_avgJ = max(max_avgJ, s_traj.avg_J);
        min_avgJ = min(min_avgJ, s_traj.avg_J);
        max_T = max(max_T, s_traj.T);
        min_T = min(min_T, s_traj.T);
      }
    }
  }

  // create d trajectories using different time of manouver and final lane
  vector<double> lanes = {2., 6., 10.};
  for (int i=0 ; i < 3 ; i++)
  {
    for (int j=0; j < 3 ; j++)
    {

      d_conds.push_back(conds_d[0]);
      d_conds.push_back(conds_d[1]);
      d_conds.push_back(conds_d[2]);
      d_conds.push_back(lanes[i]);
      d_conds.push_back(0.);
      d_conds.push_back(0.);

      double Tj = time_horizon-1;
      double dtj = (Tj - 2)/2.;

      if (Tj -dtj*j > 0)
      {
        Traj d_traj = Traj(minJerkPoly(d_conds, Tj - dtj*j), Tj - dtj*j);
        d_conds.clear();

        lateSet.push_back(d_traj);
        // useful variables for cost normalization
        double dd = abs(d_traj.getDis(d_traj.T) - lane_desired*4. - 2.);
        max_dd = max(max_dd,dd);
        min_dd = min(min_dd,dd);
        d_max_avgJ = max(d_max_avgJ, d_traj.avg_J);
        d_min_avgJ = min(d_min_avgJ, d_traj.avg_J);
        d_max_T = max(d_max_T, d_traj.T);
        d_min_T = min(d_min_T, d_traj.T);
      }
    }
  }

  vector<combiTraj> combSet; // return set
  vector<int> dyn_rej = {0,0,0}; // helepr variable to count dynamic rejections
  int coll_rej =0; // helepr variable to count collision rejections

  // set cost for every accepted trajectory and combine them
  for (int k = 0; k<longSet.size(); k++)
  {
    for (int h = 0; h<lateSet.size(); h++)
    {
      if (h==0)
      {
        double ds = abs(longSet[k].getVel(longSet[k].T) - carCtl.speed_limit);
        ds = (ds - min_ds) / (max_ds - min_ds);   // normalized distance from desired speed
        double Tc = (longSet[k].T - min_T) / (max_T - min_T); // normalized time to complete manouver
        double Jc = (longSet[k].avg_J - min_avgJ) / (max_avgJ - min_avgJ); // normalized average Jerk
        longSet[k].setCost( (10.)*Jc + (10.)*Tc + (100.)*ds ); // normalized s cost
      }

      if (k==0)
      {
        double dd = abs(lateSet[h].getDis(lateSet[h].T) - lane_desired*4. - 2.);
        dd = (dd - min_dd) / (max_dd - min_dd); // normalized distance from desired lane
        double Tc = (lateSet[h].T - d_min_T) / (d_max_T - d_min_T); // normalized time to complete manouver
        double Jc = (lateSet[h].avg_J - d_min_avgJ) / (d_max_avgJ - d_min_avgJ); // normalized average Jerk
        lateSet[h].setCost((10.)*Jc + (10.)*Tc + (10.)*dd); // normalized d cost
      }

      combiTraj comb_traj = combiTraj(longSet[k], lateSet[h], time_horizon); // combine 2 trajectories into 1 combiTraj
      int wrong_dyn = comb_traj.dynamic(carCtl.speed_limit, carCtl.acc_limit, carCtl.jerk_limit); // check for dynamic limit trespass

      if (wrong_dyn>0) {dyn_rej[wrong_dyn-1]++;}

      if ( wrong_dyn == 0 )  // if combiTraj survives dynamic check, check for collisions
      {
        bool coll = false;
        int idx = 0;

        while ( (idx < near_cars.size()) && (!coll) )
        {
          // for every near car get s,d and velocity
          double nc_s = near_cars[idx][5];
          double nc_d = near_cars[idx][6];
          double nc_vx = near_cars[idx][3];
          double nc_vy = near_cars[idx][4];
          double nc_v = sqrt(nc_vx*nc_vx + nc_vy*nc_vy);

          coll = (carCtl.trackMap->collision_test(comb_traj, {nc_s,nc_d,nc_v}, time_horizon));
          idx++;
        }
        if (!coll)
        {
          combSet.push_back(comb_traj); // if combiTraj survives the collisions check, add to the eligible trajectories set

        } else
        {

          coll_rej++;

        }
      }
      else
      {

      }
    }
  }

  return combSet;
}

vector<double> PathPlanner::keepVelPoly(vector<double> conds, double T)
{
  // This function calculates the jerk-minimizing trajectory that tries to keep a desired speed.
  // The optimal trajectory is a quartic polynomial in this case.

  // INPUT:
  //  conds is a vector containing 5 boundary conditions (s,s' and s'' at time 0 and s', s'' at time T)
  //  T is the final time for the end of trajectory

  // OUTPUT:
  // a vector of 5 coefficients for the quartic polynomial

  double s_i = conds[0];
  double s1_i = conds[1];
  double s2_i = conds[2];
  double s1_f = conds[3];
  double s2_f = conds[4];

  double A1 = s1_f - (s1_i + s2_i*T);
  double A2 = s2_f-s2_i;

  vector<double> coeffs;

  coeffs.push_back(s_i);
  coeffs.push_back(s1_i);
  coeffs.push_back(s2_i*0.5);
  coeffs.push_back( (3.*A1 - A2*T)/(3.*T*T) );
  coeffs.push_back( (A2*T - 2.*A1)/(4.*T*T*T) );

  return coeffs;
}

vector<double> PathPlanner::minJerkPoly(vector<double> conds, double T)
{
  // This function calculates the jerk-minimizing trajectory that tries to reach a final value d_f for d.
  // The optimal trajectory is a quintic polynomial in this case.

  // INPUT:
  //  conds is a vector containing 6 boundary conditions (d,d' and d'' at time 0 and d, d', d'' at time T)
  //  T is the final time for the end of trajectory

  // OUTPUT:
  // a vector of 6 coefficients for the quintic polynomial

  double d_i = conds[0];
  double d1_i = conds[1];
  double d2_i = conds[2];
  double d_f = conds[3];
  double d1_f = conds[4];
  double d2_f = conds[5];

  double A1 = (d_f - d_i - d1_i*T - 0.5*d2_i*T*T);
  double A2 = (d1_f - d1_i - d2_i*T);
  double A3 = d2_f - d2_i;

  vector<double> coeffs;

  coeffs.push_back(d_i);
  coeffs.push_back(d1_i);
  coeffs.push_back(d2_i*0.5);
  coeffs.push_back( (20.*A1 - 8.*A2*T + A3*T*T)/(2.*T*T*T) );
  coeffs.push_back( (-15.*A1 + 7.*A2*T - A3*T*T)/(T*T*T*T) );
  coeffs.push_back( (12.*A1 - 6.*A2*T + A3*T*T)/(2.*T*T*T*T*T) );

  return coeffs;
}


