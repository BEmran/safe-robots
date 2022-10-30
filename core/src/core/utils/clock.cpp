// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/clock.hpp"

namespace core::utils {

Time HighResolutionClock::Now() const {
  const std::chrono::duration<double> d = TimePoint().time_since_epoch();
  return Time{d.count()};
}

HighResolutionClock::ChronoTimePoint HighResolutionClock::TimePoint() {
  return std::chrono::high_resolution_clock::now();
}

std::shared_ptr<ClockSource> DefaultClockSource() {
  return std::make_shared<HighResolutionClock>();
}

double TimeInSeconds(std::shared_ptr<ClockSource> clock) {
  const Time now = clock->Now();
  return now.InSeconds();
}

uint64_t TimeInMicroSeconds(std::shared_ptr<ClockSource> clock) {
  const Time now = clock->Now();
  return now.InMicroSeconds();
}

std::string TimeInSecondsString(std::shared_ptr<ClockSource> clock) {
  const Time now = clock->Now();
  return now.ToString();
}

}  // namespace core::utils
