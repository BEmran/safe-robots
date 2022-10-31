// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <memory>

#include "core/utils/clock.hpp"

using core::utils::ClockSource;
using core::utils::Time;

class MockClock : public ClockSource {
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
