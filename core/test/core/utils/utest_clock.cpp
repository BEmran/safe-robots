// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <memory>
#include <string>
#include <thread>

#include "core/utils/clock.hpp"
#include "gtest/gtest.h"
#include "utils_clock.hpp"

using core::utils::ClockSource;
using core::utils::HighResolutionClock;
using core::utils::Time;
using core::utils::TimeInMicroSeconds;
using core::utils::TimeInSeconds;
using core::utils::TimeInSecondsString;
using core::utils::ToChronoDuration;

constexpr double ESTIMATED_INACCURACY_IN_SLEEP_FOR_SEC{0.0003};

TEST(MockClock, Construct) {
  MockClock clock;
  const Time time{6.7};
  clock.Set(time);
  EXPECT_DOUBLE_EQ(time, clock.Now());
}

TEST(MockClock, AsPointer) {
  MockClock clock;
  const Time time{6.7};
  clock.Set(time);
  ClockSource* clock_ptr = &clock;
  EXPECT_DOUBLE_EQ(time, clock_ptr->Now());
}

TEST(HighResolutionClock, TestWithSleepFor) {
  HighResolutionClock clock;
  const auto start = clock.Now();
  Time sleep_duration(0.1);
  std::this_thread::sleep_for(ToChronoDuration(sleep_duration));
  const auto end = clock.Now();
  const Time measured_duration = end - start;
  EXPECT_GT(measured_duration, sleep_duration);
  EXPECT_LE(measured_duration,
            sleep_duration + ESTIMATED_INACCURACY_IN_SLEEP_FOR_SEC);
}

// check TimeInSeconds function
TEST(TimeInSeconds, test) {
  auto clock = std::make_shared<MockClock>();
  const double time_in_sec = 1.234;
  const Time time{time_in_sec};
  clock->Set(time);
  EXPECT_DOUBLE_EQ(time_in_sec, TimeInSeconds(clock));
}

// check TimeInMicroSeconds function
TEST(TimeInMicroSeconds, test) {
  auto clock = std::make_shared<MockClock>();
  const double time_in_sec = 1.234;
  const Time time{time_in_sec};
  clock->Set(time);
  constexpr uint64_t Mega = 1e6;
  const uint64_t expect = static_cast<uint64_t>(time_in_sec * Mega);
  EXPECT_EQ(expect, TimeInMicroSeconds(clock));
}

// check TimeInSecondsString function
TEST(TimeInSecondsString, test) {
  auto clock = std::make_shared<MockClock>();
  const double time_in_sec = 1.234;
  const Time time{time_in_sec};
  clock->Set(time);
  EXPECT_EQ(time.ToString(), TimeInSecondsString(clock));
}
