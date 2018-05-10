//
// Created by Carey, Frank, Sr. on 2/19/18.
//
#include "gtest/gtest.h"
#include "../src/vehicle.h"
#include "../src/fsm.h"

using namespace vehicle;
using namespace utils;


//class MapTest : public ::testing::Test {
//protected:
//  virtual void SetUp() {
//    // Waypoint map to read from
//    string map_file = "../data/highway_map.csv";
//    // The max s value before wrapping around the track back to 0
//    double max_s = 6945.554;
//    const double MAX_LEGAL_MPH = 100.5; // in MPH! TODO this is actually 50.
//    const int NUMBER_OF_LANES = 3;
//    const double MAX_ACCELERATION = 8; // in meters per second.
//    trackMap = utils::Map(map_file, max_s, MAX_LEGAL_MPH, NUMBER_OF_LANES);
//  }
//
//  Map trackMap;
//};
//
//// Simple test, does not use gmock
//TEST_F(MapTest, Constructor)
//{
//  EXPECT_EQ(trackMap.n_lanes, 3);
//  EXPECT_DOUBLE_EQ(trackMap.max_s, 6945.554);
//  EXPECT_DOUBLE_EQ(trackMap.speed_limit_mph, 100.5);
//  EXPECT_EQ(trackMap.waypoints_x.size(), 181);
//  EXPECT_EQ(trackMap.waypoints_y.size(), 181);
//  EXPECT_EQ(trackMap.waypoints_dx.size(), 181);
//  EXPECT_EQ(trackMap.waypoints_dy.size(), 181);
//}
//
//
//
//
//TEST_F(MapTest, ClosestWaypoint) {
//
//  // Test that the coords for way point 8 give 8 as the closest.
//  Position pos_wp_8{1025.03, 1157.81, 0};
//  int closest_wp = trackMap.ClosestWaypoint(pos_wp_8, trackMap.waypoints_x, trackMap.waypoints_y);
//  EXPECT_EQ(closest_wp, 8);
//
//  // Test that the starting position is closest to wp 4.
//  Position pos_start{909.48, 1128.67, 0};
//  closest_wp = trackMap.ClosestWaypoint(pos_start, trackMap.waypoints_x, trackMap.waypoints_y);
//  EXPECT_EQ(closest_wp, 4);
//
//  Position somePos{930.079, 1128.85, 0.00488783};
//  closest_wp = trackMap.ClosestWaypoint(somePos, trackMap.waypoints_x, trackMap.waypoints_y);
//  EXPECT_EQ(closest_wp, 5);
//
//  Position next_and_closest_same{927.479, 1128.82, 0.00488783};
//  closest_wp = trackMap.ClosestWaypoint(next_and_closest_same, trackMap.waypoints_x, trackMap.waypoints_y);
//  EXPECT_EQ(closest_wp, 5);
//
//}
//
//TEST_F(MapTest, NextWaypoint) {
//
//  // Test that the coords for way point 8 give 8 as the closest.
//  Position pos_wp_8{1025.03, 1157.81, 0};
//  int next_wp = trackMap.NextWaypoint(pos_wp_8, trackMap.waypoints_x, trackMap.waypoints_y);
//  EXPECT_EQ(next_wp, 9);
//
//  // Test that the starting position is closest to wp 4.
//  Position pos_start{909.48, 1128.67, 0};
//  next_wp = trackMap.NextWaypoint(pos_start, trackMap.waypoints_x, trackMap.waypoints_y);
//  EXPECT_EQ(next_wp, 5);
//
//  Position somePos{930.079, 1128.85, 0.00488783};
//  next_wp = trackMap.NextWaypoint(somePos, trackMap.waypoints_x, trackMap.waypoints_y);
//  EXPECT_EQ(next_wp, 5);
//
//  Position next_and_closest_same{927.479, 1128.82, 0.00488783};
//  next_wp = trackMap.NextWaypoint(next_and_closest_same, trackMap.waypoints_x, trackMap.waypoints_y);
//  EXPECT_EQ(next_wp, 5);
//
//}
//
//TEST_F(MapTest, getFrenet) {
//  // Starting point of the map when staring the simulator.
//  Position startPos{909.48000000000001, 1128.6700000000001, 0};
//  FrenetPos startFrenet = trackMap.getFrenet(startPos);
//
//  EXPECT_NEAR(startFrenet.s, 124.8336, 0.001);
//  EXPECT_NEAR(startFrenet.d, 6.1648, 0.001);
//
//
//  Position somePos{930.079, 1128.85, 0.00488783};
//  FrenetPos someFrenet = trackMap.getFrenet(somePos);
//
//  EXPECT_NEAR(someFrenet.s, 145.434, 0.001);
//  EXPECT_NEAR(someFrenet.d, 6.1648, 0.003);
//
//
//  Position trickyPos{2275.37, 2311.45, 1.36312};
//  FrenetPos trickyFrenet = trackMap.getFrenet(trickyPos);
//
//  EXPECT_NEAR(trickyFrenet.s, 2389.84, 0.001);
//  EXPECT_NEAR(trickyFrenet.d, 4.97685, 0.003);
//
//}