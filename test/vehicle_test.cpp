//
// Created by Carey, Frank, Sr. on 2/19/18.
//
#include "gtest/gtest.h"
#include "../src/vehicle.h"
#include "../src/fsm.h"

using namespace vehicle;
using namespace utils;

class VehicleControllerTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    // Waypoint map to read from
    string map_file = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;
    const double MAX_LEGAL_MPH = 100.5; // in MPH! TODO this is actually 50.
    const int NUMBER_OF_LANES = 3;
    const double MAX_ACCELERATION = 8; // in meters per second.
    trackMap = utils::Map(map_file, max_s, MAX_LEGAL_MPH, NUMBER_OF_LANES);
    v = Vehicle(0, Position{909.48, 1128.67, 0});
  }
  Map trackMap;
  fsm::VehicleFSM fsm{};
  Vehicle v;

};

TEST_F(VehicleControllerTest, getLaneKinematic)
{
  VehicleController carCtl {v, &fsm, &trackMap};

  Vehicle curr_vpt = v.clone();

  // Assume there are no other vehicles on the road.
  map<int, vector<Vehicle>> other_vehicles{};

  // Stay in lane 1 with a target velocity of 1 m/s and a time delta of 1s.
  Vehicle new_vpt = carCtl.get_lane_kinematic(curr_vpt, 1, 1., 1., other_vehicles);

  // Since we're starting at 0 m/s and want to end a 1 m/s,
  // We'll only travel 0.5 m in the first second. This yaw means it's all x.
  EXPECT_NEAR(carCtl.vehicle.x() + .5, new_vpt.x(), 0.01);
  // But our velocty now should be 1 m/s.
  EXPECT_NEAR(carCtl.vehicle.v() + 1., new_vpt.v(), 0.01);
  // With the yaw we had, there shouldn't be change to y really.
  EXPECT_NEAR(carCtl.vehicle.y(), new_vpt.y(), 0.01);
  // Time should be one more in the new vpt.
  EXPECT_NEAR(carCtl.vehicle.secondsDiff(new_vpt.time()), 1, 0.0001);
}

TEST_F(VehicleControllerTest, extendTrajectory)
{
  VehicleController carCtl {v, &fsm, &trackMap};

  Vehicle curr_vpt = v.clone();

  vector<Vehicle> path{curr_vpt};

  for(int i=0; i<4; i++) {
    curr_vpt.x(curr_vpt.x() + 1);
    curr_vpt.y(curr_vpt.y());
    curr_vpt.addSeconds(1/4.);
    path.push_back(curr_vpt);
    curr_vpt = curr_vpt.clone();
  }

  carCtl.extend_trajectory(path);

  EXPECT_EQ(carCtl.trajectory.size(), 49);
  Vehicle half_vpt = carCtl.trajectory[24];
  Vehicle final_vpt = carCtl.trajectory[48];

  // The trajectory x() value should increase by 4 over one second.
  // 1 second ~= 48 inc * .02 seconds.
  EXPECT_NEAR(final_vpt.x(), v.x() + 4, 0.1);

  // Half way pt should have changed by half that.
  EXPECT_NEAR(half_vpt.x(), v.x() + 2, 0.1);

  // Y shouldn't have changed though.
  EXPECT_NEAR(final_vpt.y(), v.y(), 0.001);

}


// Simple test, does not use gmock
TEST(Vehicle, Constructor)
{
  time_point<system_clock> pre_time = system_clock::now();


  Vehicle v{};

  EXPECT_DOUBLE_EQ(v.x(),0);
  EXPECT_DOUBLE_EQ(v.y(),0);
  EXPECT_DOUBLE_EQ(v.yaw(),0);
  EXPECT_DOUBLE_EQ(v.yaw_delta(),0);
  EXPECT_DOUBLE_EQ(v.a(),0);
  EXPECT_DOUBLE_EQ(v.v(),0);

  // The default time should be recent.
  EXPECT_LE(v.secondsDiff(pre_time), 0);
  EXPECT_GT(v.secondsDiff(pre_time), -1);

  // The two time items should not point to the same memory (they should be copies)
  time_point<chrono::system_clock> temp_time_1 = v.time();
  time_point<chrono::system_clock> temp_time_2 = v.time();

  EXPECT_NE(&temp_time_1, &temp_time_2);


  Vehicle v2(0, Position{});
  // The default time should be recent.
  EXPECT_LE(v2.secondsDiff(pre_time), 0);
  EXPECT_GT(v2.secondsDiff(pre_time), -1);



}

// Simple test, does not use gmock
TEST(Vehicle, time)
{
  Vehicle v{};
  time_point<chrono::system_clock> orig_time = v.time();

  v.addSeconds(10);
  // Durations are in microseconds (not milliseconds).
  double diff = (v.time() - orig_time).count() / 1000000.;

  // The default time should be recent.
  EXPECT_EQ(diff,  10);

  v.addSeconds(2.5);
  EXPECT_DOUBLE_EQ(v.secondsDiff(orig_time), -12.5);

}

TEST(VehicleController, Constructor)
{
  // Waypoint map to read from
  string map_file = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  const double MAX_LEGAL_MPH = 100.5; // in MPH! TODO this is actually 50.
  const int NUMBER_OF_LANES = 3;
  const double MAX_ACCELERATION = 8; // in meters per second.


  utils::Map trackMap =  utils::Map(map_file, max_s, MAX_LEGAL_MPH, NUMBER_OF_LANES);
  Vehicle v{0, Position{1,2,3}};
  fsm::VehicleFSM fsm{};

  VehicleController carCtl = VehicleController(v, &fsm, &trackMap);

  // Vehicle should be copied.
  EXPECT_NE(&v, &(carCtl.vehicle));
  EXPECT_DOUBLE_EQ(carCtl.vehicle.x(), 1);
  EXPECT_DOUBLE_EQ(carCtl.vehicle.y(), 2);
  EXPECT_DOUBLE_EQ(carCtl.vehicle.yaw(), 3);


  // trackMap and fsm should use pointers.
  EXPECT_EQ(&trackMap, carCtl.trackMap);
  EXPECT_EQ(&fsm, carCtl.fsm);

}


//TEST(Points, Constructor)
//{
//  Position pt = Position();
//
//  EXPECT_DOUBLE_EQ(pt.x, 0);
//  EXPECT_DOUBLE_EQ(pt.y, 0);
//  EXPECT_DOUBLE_EQ(pt.yaw, 0);
//}
//
//TEST(Points, ConvertToFrame) {
//
//  Position world_pt = Position();
//  Position car = Position(-1, -1, deg2rad(90)); // Facing up.
//
//  Position relative_pt = world_pt.convert_to_frame(car);
//  EXPECT_DOUBLE_EQ(relative_pt.x, 1);
//  EXPECT_DOUBLE_EQ(relative_pt.y, -1);
//  EXPECT_DOUBLE_EQ(rad2deg(relative_pt.yaw), 270.); // Facing down.
//}
//
//
//TEST(Points, ConvertFromFrame) {
//
//  Position car_frame_pt = Position(1, -1, deg2rad(270)); // Facing down.
//  Position car = Position(-1, -1, deg2rad(90)); // Facing up.
//
//  Position world_pt =  car_frame_pt.convert_from_frame(car);
//  EXPECT_NEAR(world_pt.x, 0,  0.0000001);
//  EXPECT_NEAR(world_pt.y, 0,  0.0000001);
//  EXPECT_DOUBLE_EQ(rad2deg(world_pt.yaw), 0); // Facing right.
//}
//
//TEST(Spline, CreateStraightLine) {
//  vector<Position> way_pts;
//  for (int i=0; i < 3; i++) {
//    way_pts.push_back(Position(i, 0, 0));
//  }
//  Spline spl = Spline(way_pts);
//
//  for (int i=0; i < 5; i++) {
//    Position i_pt = spl.interpolate(i);
//    ASSERT_NEAR(i_pt.y, 0,  0.0000001);
//  }
//}
//
//
//TEST(Spline, CreateDiagonalLine) {
//  vector<Position> way_pts;
//  for (int i=0; i < 5; i++) {
//    way_pts.push_back(Position(i, i, 0));
//  }
//  Spline spl = Spline(way_pts);
//
//  for (int i=0; i < 5; i++) {
//    Position i_pt = spl.interpolate(i);
//    EXPECT_NEAR(i_pt.y, i, i * 0.001);
//  }
//}
//
//
//TEST(Spline, CreateCurvedLine) {
//  vector<Position> way_pts;
//  for (int i=0; i < 5; i++) {
//    way_pts.push_back(Position(i, i*i, 0));
//  }
//  Spline spl = Spline(way_pts);
//
//  // Note, the values can be off like 3-5%, especially near the ends of the spline!
//  for (int i=2; i < 10; i++) {
//    double half_i = i / 2.;
//    Position i_pt = spl.interpolate(half_i);
//    EXPECT_NEAR(i_pt.y, half_i*half_i, half_i*half_i * 0.03);
//  }
//}
