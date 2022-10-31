// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/timer.hpp"

#include "core/utils/clock.hpp"

namespace core::utils {

Timer::Timer(const Time& duration) : Timer(duration, DefaultClockSource()) {
}

Timer::Timer(const Time& duration, std::shared_ptr<ClockSource> clock)
  : duration_{duration}, clock_{clock}, ptime_{clock->Now()} {
}

void Timer::SetDuration(const Time& duration) {
  duration_ = duration.InSeconds();
}

void Timer::ResetTime() {
  ptime_ = clock_->Now();
}

bool Timer::IsTime() {
  const Time ctime = clock_->Now();
  const double tmp_duration = ctime - ptime_;
  if (tmp_duration >= duration_) {
    ptime_ = ctime;
    return true;
  }
  return false;
}

}  // namespace core::utils