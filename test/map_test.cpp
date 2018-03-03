//
// Created by Carey, Frank, Sr. on 2/19/18.
//
#include "gtest/gtest.h"
#include "../src/vehicle.h"
#include "../src/fsm.h"

using namespace vehicle;
using namespace utils;


class MapTest : public ::testing::Test {
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
  }

  Map trackMap;
};

// Simple test, does not use gmock
TEST_F(MapTest, Constructor)
{
  EXPECT_EQ(trackMap.n_lanes, 3);
  EXPECT_DOUBLE_EQ(trackMap.max_s, 6945.554);
  EXPECT_DOUBLE_EQ(trackMap.speed_limit_mph, 100.5);
  EXPECT_EQ(trackMap.waypoints_x.size(), 181);
  EXPECT_EQ(trackMap.waypoints_y.size(), 181);
  EXPECT_EQ(trackMap.waypoints_dx.size(), 181);
  EXPECT_EQ(trackMap.waypoints_dy.size(), 181);
}




TEST_F(MapTest, ClosestWaypoint) {

  // Test that the coords for way point 8 give 8 as the closest.
  Position pos_wp_8{1025.03, 1157.81, 0};
  int closest_wp = trackMap.ClosestWaypoint(pos_wp_8);
  EXPECT_EQ(closest_wp, 8);

  // Test that the starting position is closest to wp 4.
  Position pos_start{909.48, 1128.67, 0};
  closest_wp = trackMap.ClosestWaypoint(pos_start);
  EXPECT_EQ(closest_wp, 4);
  
}

TEST_F(MapTest, getFrenet) {
  // Starting point of the map when staring the simulator.
  Position pos{909.48000000000001, 1128.6700000000001, 0};
  FrenetPos frenet = trackMap.getFrenet(pos);

  EXPECT_NEAR(frenet.s, 124.8336, 0.0003);
  EXPECT_NEAR(frenet.d, 6.1648, 0.0003);

}