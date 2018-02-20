//
// Created by Carey, Frank, Sr. on 2/19/18.
//
#include "gtest/gtest.h"
#include "../src/utils.h"

using namespace utils;


// Simple test, does not use gmock
TEST(Utils, Conversions)
{
  double meters_per_sec = from_mph(123);
  EXPECT_NEAR(meters_per_sec, 54.98592, 0.00001);

  double mph = to_mph(54.98592);
  EXPECT_DOUBLE_EQ(mph, 123);

}

TEST(Points, Constructor)
{
  Point pt = Point();

  EXPECT_DOUBLE_EQ(pt.x, 0);
  EXPECT_DOUBLE_EQ(pt.y, 0);
  EXPECT_DOUBLE_EQ(pt.yaw, 0);
}

TEST(Points, ConvertToFrame) {

  Point world_pt = Point();
  Point car = Point(-1, -1, deg2rad(90)); // Facing up.

  Point relative_pt = world_pt.convert_to_frame(car);
  EXPECT_DOUBLE_EQ(relative_pt.x, 1);
  EXPECT_DOUBLE_EQ(relative_pt.y, -1);
  EXPECT_DOUBLE_EQ(rad2deg(relative_pt.yaw), 270.); // Facing down.
}


TEST(Points, ConvertFromFrame) {

  Point car_frame_pt = Point(1, -1, deg2rad(270)); // Facing down.
  Point car = Point(-1, -1, deg2rad(90)); // Facing up.

  Point world_pt =  car_frame_pt.convert_from_frame(car);
  EXPECT_NEAR(world_pt.x, 0,  0.0000001);
  EXPECT_NEAR(world_pt.y, 0,  0.0000001);
  EXPECT_DOUBLE_EQ(rad2deg(world_pt.yaw), 0); // Facing right.
}

TEST(Spline, CreateStraightLine) {
  vector<Point> way_pts;
  for (int i=0; i < 3; i++) {
    way_pts.push_back(Point(i, 0, 0));
  }
  Spline spl = Spline(way_pts);

  for (int i=0; i < 5; i++) {
    Point i_pt = spl.interpolate(i);
    ASSERT_NEAR(i_pt.y, 0,  0.0000001);
  }
}


TEST(Spline, CreateDiagonalLine) {
  vector<Point> way_pts;
  for (int i=0; i < 5; i++) {
    way_pts.push_back(Point(i, i, 0));
  }
  Spline spl = Spline(way_pts);

  for (int i=0; i < 5; i++) {
    Point i_pt = spl.interpolate(i);
    EXPECT_NEAR(i_pt.y, i, i * 0.001);
  }
}


TEST(Spline, CreateCurvedLine) {
  vector<Point> way_pts;
  for (int i=0; i < 5; i++) {
    way_pts.push_back(Point(i, i*i, 0));
  }
  Spline spl = Spline(way_pts);

  // Note, the values can be off like 3-5%, especially near the ends of the spline!
  for (int i=2; i < 10; i++) {
    double half_i = i / 2.;
    Point i_pt = spl.interpolate(half_i);
    EXPECT_NEAR(i_pt.y, half_i*half_i, half_i*half_i * 0.03);
  }
}
