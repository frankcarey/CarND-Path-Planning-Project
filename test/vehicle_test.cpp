//
// Created by Carey, Frank, Sr. on 2/19/18.
//
#include "gtest/gtest.h"
#include "../src/vehicle.h"

using namespace vehicle;
using namespace utils;


// Simple test, does not use gmock
TEST(Vehicle, Constructor)
{
  Vehicle(Position(0, 0, 0));
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
