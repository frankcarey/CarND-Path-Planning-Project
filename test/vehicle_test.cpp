//
// Created by Carey, Frank, Sr. on 2/19/18.
//
#include "gtest/gtest.h"
#include "../src/vehicle.h"
#include "../src/fsm.h"

using namespace vehicle;
using namespace utils;


// Simple test, does not use gmock
TEST(Vehicle, Constructor)
{
  double pre_time = system_clock::now().time_since_epoch().count() - 1;

  Vehicle v{};

  EXPECT_DOUBLE_EQ(v.x(),0);
  EXPECT_DOUBLE_EQ(v.y(),0);
  EXPECT_DOUBLE_EQ(v.yaw(),0);
  EXPECT_DOUBLE_EQ(v.yaw_delta(),0);
  EXPECT_DOUBLE_EQ(v.a(),0);
  EXPECT_DOUBLE_EQ(v.v(),0);

  // The default time should be recent.
  EXPECT_GT(v.time().time_since_epoch().count() , pre_time);
  EXPECT_LT(v.time().time_since_epoch().count() , system_clock::now().time_since_epoch().count());

  // The two time items should not point to the same memory (they should be copies)
  time_point<chrono::system_clock> temp_time_1 = v.time();
  time_point<chrono::system_clock> temp_time_2 = v.time();

  EXPECT_NE(&temp_time_1, &temp_time_2);


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
