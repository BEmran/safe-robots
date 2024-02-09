// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <climits>

#include "core/utils/min_max_statistics.hpp"
#include "gtest/gtest.h"

using core::utils::MinMaxStatistics;

TEST(MinMaxStatistics, InitialMinMaxValue) {
  MinMaxStatistics<int> sat;
  EXPECT_EQ(std::numeric_limits<int>::max(), sat.min_value);
  EXPECT_EQ(std::numeric_limits<int>::min(), sat.max_value);
  EXPECT_EQ(0, sat.counter);
  EXPECT_DOUBLE_EQ(0.0, sat.sum);
}

TEST(MinMaxStatistics, MinMaxAreSameFirstUpdate) {
  MinMaxStatistics<int> sat;
  sat.Update(1);
  EXPECT_EQ(1, sat.max_value);
  EXPECT_EQ(1, sat.min_value);
  EXPECT_EQ(1, sat.counter);
  EXPECT_DOUBLE_EQ(1.0, sat.sum);
}

TEST(MinMaxStatistics, UpdateTwoSameValues) {
  MinMaxStatistics<int> sat;
  sat.Update(1);
  sat.Update(1);
  EXPECT_EQ(1, sat.min_value);
  EXPECT_EQ(1, sat.max_value);
  EXPECT_EQ(2, sat.counter);
  EXPECT_DOUBLE_EQ(2.0, sat.sum);
}

TEST(MinMaxStatistics, UpdateTwoDistinguishValues) {
  MinMaxStatistics<int> sat;
  sat.Update(1);
  sat.Update(2);
  EXPECT_EQ(1, sat.min_value);
  EXPECT_EQ(2, sat.max_value);
  EXPECT_EQ(2, sat.counter);
  EXPECT_DOUBLE_EQ(3.0, sat.sum);
}

TEST(MinMaxStatistics, ResetMinMaxValueAfterUpdate) {
  MinMaxStatistics<int> sat;
  sat.Update(1);
  EXPECT_EQ(1, sat.min_value);
  EXPECT_EQ(1, sat.max_value);
  EXPECT_EQ(1, sat.counter);
  EXPECT_EQ(1, sat.sum);
  sat.ResetMinMax();
  EXPECT_EQ(0, sat.counter);
  EXPECT_DOUBLE_EQ(0.0, sat.sum);
  EXPECT_EQ(std::numeric_limits<int>::max(), sat.min_value);
  EXPECT_EQ(std::numeric_limits<int>::min(), sat.max_value);
}

TEST(MinMaxStatistics, CalculateAverage) {
  MinMaxStatistics<int> sat;
  sat.Update(1);
  sat.Update(2);
  EXPECT_EQ(1, sat.min_value);
  EXPECT_EQ(2, sat.max_value);
  EXPECT_EQ(2, sat.counter);
  EXPECT_DOUBLE_EQ(3.0, sat.sum);
  EXPECT_DOUBLE_EQ(1.5, sat.CalculateAverage());
}

TEST(MinMaxStatistics, CalculateAverageWithoutUpdate) {
  MinMaxStatistics<int> sat;
  EXPECT_DOUBLE_EQ(0.0, sat.CalculateAverage());
}

TEST(MinMaxStatistics, Stream) {
  MinMaxStatistics<int> sat;
  sat.Update(1);
  sat.Update(2);
  std::stringstream ss;
  ss << sat;
  std::string expect = "average: 1.5 min: 1 max: 2";
  EXPECT_EQ(expect, ss.str());
}
