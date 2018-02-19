//
// Created by Carey, Frank, Sr. on 2/19/18.
//
#include "gtest/gtest.h"
#include "../src/utils.h"


// Simple test, does not use gmock
TEST(Dummy, foobar)
{
  float meters_per_sec = from_mph(123);
  ASSERT_FLOAT_EQ(meters_per_sec, 54.98592);
}