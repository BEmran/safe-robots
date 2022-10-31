// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <string>

#include "core/utils/timer.hpp"
#include "gtest/gtest.h"
#include "utest/utils_clock.hpp"

using core::utils::Timer;

TEST(Timer, IsTimeReturnFalse) {
  auto clock = std::make_shared<MockClock>();
  clock->Set(Time{0.0});
  Time duration{1.0};
  Timer timer(duration, clock);
  EXPECT_FALSE(timer.IsTime());
  clock->Set(0.5);
  EXPECT_FALSE(timer.IsTime());
}

TEST(Timer, IsTimeReturnTrue) {
  auto clock = std::make_shared<MockClock>();
  clock->Set(Time{0.0});
  Time duration{1.0};
  Timer timer(duration, clock);
  EXPECT_FALSE(timer.IsTime());
  clock->Set(1.0);
  EXPECT_TRUE(timer.IsTime());
}

TEST(Timer, IsTimeReturnTrueMultipleTime) {
  auto clock = std::make_shared<MockClock>();
  clock->Set(Time{0.0});
  Time duration{1.0};
  Timer timer(duration, clock);
  EXPECT_FALSE(timer.IsTime());
  clock->Set(1.0);
  EXPECT_TRUE(timer.IsTime());
  clock->Set(1.5);
  EXPECT_FALSE(timer.IsTime());
  clock->Set(2.0);
  EXPECT_TRUE(timer.IsTime());
}

TEST(Timer, IsTimeReturnFalseAfterReset) {
  auto clock = std::make_shared<MockClock>();
  clock->Set(Time{0.0});
  Time duration{2.0};
  Timer timer(duration, clock);
  EXPECT_FALSE(timer.IsTime());
  clock->Set(2.0);
  timer.ResetTime();
  EXPECT_FALSE(timer.IsTime());
}

TEST(Timer, SetDuration) {
  auto clock = std::make_shared<MockClock>();
  clock->Set(Time{0.0});
  Time duration{2.0};
  Timer timer(duration, clock);
  timer.SetDuration(Time{5.0});
  clock->Set(2.0);
  EXPECT_FALSE(timer.IsTime());
  clock->Set(5.0);
  EXPECT_TRUE(timer.IsTime());
}
