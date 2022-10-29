// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_CLOCK_HPP_
#define CORE_UTILS_CLOCK_HPP_

#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>

namespace core::utils {

namespace {
using Duration = std::chrono::duration<double, std::ratio<1>>;
}  // namespace

struct TimeComponent {
  long seconds{0};
  long micros{0};

  TimeComponent();
  TimeComponent(const long sec, const long micro);
  explicit TimeComponent(const double time_in_sec);
  double ToSeconds() const;
};

class Time {
 public:
  Time();

  Time(const double time_in_sec);

  explicit Time(const TimeComponent& tc);

  TimeComponent Component() const;

  double InSeconds() const;

  uint64_t InMicroSeconds() const;

  Duration ToChronoDuration() const;

  operator double() const;

  std::string ToString() const;

  std::string Pretty() const;

 private:
  double in_sec_{0.0};
};

std::ostream& operator<<(std::ostream& os, const Time& time);

class ClockInterface {
 public:
  virtual ~ClockInterface() = default;
  virtual Time Now() const = 0;
};

class HighResolutionClock : public ClockInterface {
  using ChronoTimePoint = std::chrono::system_clock::time_point;

 public:
  Time Now() const override;
  static ChronoTimePoint TimePoint();
};

double TimeInSeconds(std::unique_ptr<ClockInterface> clock =
                       std::make_unique<HighResolutionClock>());

uint64_t TimeInMicroSeconds(std::unique_ptr<ClockInterface> clock =
                              std::make_unique<HighResolutionClock>());

std::string TimeInSecondsString(std::unique_ptr<ClockInterface> clock =
                                  std::make_unique<HighResolutionClock>());

}  // namespace core::utils

#endif  // CORE_UTILS_CLOCK_HPP_