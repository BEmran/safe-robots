// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <string>

#include "core/utils/time.hpp"
#include "gtest/gtest.h"

using core::utils::Time;
using core::utils::TimeComponent;
using core::utils::ToChronoDuration;

TEST(TimeComponent, DefaultConstruct) {
  const TimeComponent tc;
  EXPECT_EQ(0, tc.seconds);
  EXPECT_EQ(0, tc.micros);
}

TEST(TimeComponent, SeparateConstruct) {
  constexpr long sec = 3;
  constexpr long micro = 6;
  const TimeComponent tc{sec, micro};
  EXPECT_EQ(sec, tc.seconds);
  EXPECT_EQ(micro, tc.micros);
}

TEST(TimeComponent, ConstructUsingTimeInSec) {
  constexpr double time_in_sec = 3.4;
  const TimeComponent tc{time_in_sec};
  EXPECT_EQ(3, tc.seconds);
  EXPECT_EQ(4e5, tc.micros);
}

TEST(TimeComponent, ToSeconds) {
  constexpr double time_in_sec = 3.4;
  const TimeComponent tc{time_in_sec};
  EXPECT_DOUBLE_EQ(time_in_sec, tc.ToSeconds());
}

TEST(Time, DefaultConstruct) {
  const Time time;
  EXPECT_DOUBLE_EQ(0.0, time.InSeconds());
  EXPECT_EQ(0, time.InMicroSeconds());
}

TEST(Time, ConstructUsingTimeInSec) {
  constexpr double time_in_sec = 1.02;
  Time time(time_in_sec);
  EXPECT_DOUBLE_EQ(time_in_sec, time.InSeconds());
  const uint32_t expect = static_cast<uint32_t>(time_in_sec * 1e6);
  EXPECT_EQ(expect, time.InMicroSeconds());
}

TEST(Time, ConstructUsingTimeComponent) {
  constexpr double time_in_sec = 3.4;
  const TimeComponent tc{time_in_sec};
  Time time(tc);
  EXPECT_DOUBLE_EQ(time_in_sec, time.InSeconds());
  EXPECT_EQ(tc.seconds, time.Component().seconds);
  EXPECT_EQ(tc.micros, time.Component().micros);
}

TEST(Time, CheckAdding) {
  double time_in_sec = 12.5;
  Time time(time_in_sec);
  double extra_time_in_sec = 1.2;
  double expect = time_in_sec + extra_time_in_sec;
  EXPECT_DOUBLE_EQ(expect, time + extra_time_in_sec);
}

TEST(Time, CheckMultiplying) {
  double time_in_sec = 5.2;
  Time time(time_in_sec);
  double scale = 4;
  double expect = time_in_sec * scale;
  EXPECT_DOUBLE_EQ(expect, time * scale);
}

TEST(Time, ConstructNewTimeAfterArithmetic) {
  double time_in_sec = 5.2;
  double scale = 4;
  double addition = 3;
  Time time(time_in_sec);
  Time result = time * scale + addition;
  double expect = time_in_sec * scale + addition;
  EXPECT_DOUBLE_EQ(expect, result);
}

TEST(Time, ToString) {
  double time_in_sec = 5.2;
  Time time(time_in_sec);
  std::string expect = std::to_string(time.InSeconds());
  EXPECT_EQ(expect, time.ToString());
}

TEST(Time, Pretty) {
  double time_in_sec = 5.2;
  Time time(time_in_sec);
  std::string expect = std::to_string(time.InSeconds()) + " [sec]";
  EXPECT_EQ(expect, time.Pretty());
}

TEST(Time, Stream) {
  double time_in_sec = 5.2;
  Time time(time_in_sec);
  time = 2;
  std::stringstream ss;
  ss << time;
  EXPECT_EQ(time.ToString(), ss.str());
}

TEST(ToChronoDuration, test) {
  double time_in_sec = 5.2;
  Time time(time_in_sec);
  EXPECT_DOUBLE_EQ(time_in_sec, ToChronoDuration(time).count());
}