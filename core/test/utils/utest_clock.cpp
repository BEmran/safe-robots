// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <string>
#include <thread>

#include "core/utils/clock.hpp"
#include "gtest/gtest.h"

using core::utils::ClockInterface;
using core::utils::HighResolutionClock;
using core::utils::Time;
using core::utils::TimeComponent;

constexpr double INACCURACY_IN_SLEEP_FOR_SEC{0.0002};

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
  constexpr double in_sec = 3.4;
  const TimeComponent tc{in_sec};
  EXPECT_EQ(3, tc.seconds);
  EXPECT_EQ(4e5, tc.micros);
}

TEST(TimeComponent, InSec) {
  constexpr double in_sec = 3.4;
  const TimeComponent tc{in_sec};
  EXPECT_DOUBLE_EQ(in_sec, tc.ToSec());
}

TEST(Time, DefaultConstruct) {
  const Time time_struct;
  EXPECT_DOUBLE_EQ(0.0, time_struct.InSec());
}

TEST(Time, ConstructUsingTimeInSec) {
  constexpr double time_in_sec = 1.02;
  Time time_struct(time_in_sec);
  EXPECT_DOUBLE_EQ(time_in_sec, time_struct.InSec());
}

TEST(Time, Component) {
  constexpr long sec = 3;
  constexpr long micro = 5;
  const TimeComponent tc{sec, micro};
  Time time_struct(tc.ToSec());
  EXPECT_EQ(sec, time_struct.Component().seconds);
  EXPECT_EQ(micro, time_struct.Component().micros);
}

TEST(Time, CheckAdding) {
  double start_time_in_sec = 12.5;
  Time time_struct(start_time_in_sec);
  double extra_time_in_sec = 1.2;
  double expect = start_time_in_sec + extra_time_in_sec;
  EXPECT_DOUBLE_EQ(expect, time_struct + extra_time_in_sec);
}

TEST(Time, CheckMultiplying) {
  double start_time_in_sec = 5.2;
  Time time_struct(start_time_in_sec);
  double scale = 4;
  double expect = start_time_in_sec * scale;
  EXPECT_DOUBLE_EQ(expect, time_struct * scale);
}

TEST(Time, ConstructNewTimeAfterArithmetic) {
  double start_time_in_sec = 5.2;
  double scale = 4;
  double addition = 3;
  Time time_struct(start_time_in_sec);
  Time result = time_struct * scale + addition;
  double expect = start_time_in_sec * scale + addition;
  EXPECT_DOUBLE_EQ(expect, result);
}

TEST(Time, ToString) {
  double start_time_in_sec = 5.2;
  Time time_struct(start_time_in_sec);
  std::string expect = std::to_string(time_struct.InSec());
  EXPECT_EQ(expect, time_struct.ToString());
}

TEST(Time, Pretty) {
  double start_time_in_sec = 5.2;
  Time time_struct(start_time_in_sec);
  std::string expect = std::to_string(time_struct.InSec()) + " [sec]";
  EXPECT_EQ(expect, time_struct.Pretty());
}

TEST(Time, Stream) {
  double start_time_in_sec = 5.2;
  Time time_struct(start_time_in_sec);
  time_struct = 2;
  std::stringstream ss;
  ss << time_struct;
  EXPECT_EQ(time_struct.ToString(), ss.str());
}

class MockClock : public ClockInterface {
 public:
  Time Now() const override {
    return time_;
  }
  void Set(const Time& time) {
    time_ = time;
  }

 private:
  Time time_;
};

TEST(MockClock, Construct) {
  const Time time{6.7};
  MockClock clock;
  clock.Set(time);
  EXPECT_DOUBLE_EQ(time, clock.Now());
}

TEST(MockClock, AsPointer) {
  const Time time{6.7};
  MockClock clock;
  clock.Set(time);
  ClockInterface* clock_ptr = &clock;
  EXPECT_DOUBLE_EQ(time, clock_ptr->Now());
}

TEST(HighResolutionClock, TestWithSleepFor) {
  HighResolutionClock clock;
  const auto start = clock.Now();
  Time sleep_duration(0.1);
  std::this_thread::sleep_for(sleep_duration.ToChronoDuration());
  const auto end = clock.Now();
  const Time measured_duration = end - start;
  EXPECT_GT(measured_duration, sleep_duration);
  EXPECT_LE(measured_duration, sleep_duration + INACCURACY_IN_SLEEP_FOR_SEC);
}
