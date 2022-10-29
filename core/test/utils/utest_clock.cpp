// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <cmath>
#include <memory>
#include <string>

#include "core/utils/clock.hpp"
#include "gtest/gtest.h"

using core::utils::ClockInterface;
using core::utils::Time;
using core::utils::TimeComponent;
using core::utils::TimeInterface;

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

class SimpleTime : public TimeInterface {
 public:
  SimpleTime(double time) : time_(time) {
  }

  Time GetTime() const override {
    return Time(time_);
  }

 private:
  double time_;
};

TEST(SimpleTime, Construct) {
  constexpr double in_sec = 6.7;
  const SimpleTime st(in_sec);
  EXPECT_DOUBLE_EQ(in_sec, st.GetTime().InSec());
}

TEST(SimpleTime, AsPointer) {
  constexpr double in_sec = 6.7;
  const SimpleTime st(in_sec);
  const TimeInterface* st_ptr = &st;
  EXPECT_DOUBLE_EQ(in_sec, st_ptr->GetTime().InSec());
}

// class MockClock : public ClockInterface {
//  public:
//   std::unique_ptr<TimeInterface> Now() override {
//     return std::make_unique<SimpleTime>(time_);
//   }
//   Time TimeNow() override {
//     return Now()->Time();
//   }
//   void Set(Time time) {
//     time_ = time;
//   }

//  private:
//   Time time_;
// };

// TEST(Clock, ) {
// }
// ::testing::AssertionResult ExpectGPSData(GpsData& expect,
//                                          GpsData& actual) {
//   auto e1 = ExpectEq(expect.lat, actual.lat, "Latitude value:");
//   auto e2 = ExpectEq(expect.lon, actual.lon, "Longitude value:");
//   auto e3 = ExpectEq(expect.alt, actual.alt, "Altitude value:");
//   return e1 && e2 && e3;
// }
