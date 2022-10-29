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

constexpr long MEGA = 1e6;
constexpr double MICRO = 1e-6;

}  // namespace

struct TimeComponent {
  long seconds{0};
  long micros{0};

  TimeComponent() : seconds{0}, micros{0} {
  }

  TimeComponent(const long sec, const long micro)
    : seconds{sec}, micros{micro} {
  }

  TimeComponent(const double time_in_sec) {
    seconds = static_cast<long>(time_in_sec);
    micros = static_cast<long>(time_in_sec * MEGA) - seconds * MEGA;
  }

  double ToSec() const {
    return static_cast<double>(seconds) + static_cast<double>(micros) * MICRO;
  }
};

class Time {
 public:
  Time() : in_sec_{0.0} {
  }

  Time(const double time_in_sec) : in_sec_(time_in_sec) {
  }

  inline TimeComponent Component() const {
    return TimeComponent(in_sec_);
  }

  inline double InSec() const {
    return in_sec_;
  }

  Duration ToChronoDuration() const {
    return Duration{in_sec_};
  }

  inline operator double() const {
    return in_sec_;
  }

  inline std::string ToString() const {
    return std::to_string(in_sec_);
  }

  inline std::string Pretty() const {
    return ToString() + " [sec]";
  }

 private:
  double in_sec_{0.0};
};

std::ostream& operator<<(std::ostream& os, const Time& time) {
  return os << time.ToString();
}

class TimeInterface {
 public:
  virtual ~TimeInterface() = default;
  virtual Time GetTime() const = 0;
};

class ChronoTime : public TimeInterface {
  using TimePoint = std::chrono::system_clock::time_point;

 public:
  ChronoTime(TimePoint tp) : tp_(tp) {
  }

  TimePoint GetTimePoint() const {
    return tp_;
  }

  TimePoint& GetTimePoint() {
    return tp_;
  }

  Time GetTime() const override {
    Duration d{tp_.time_since_epoch()};
    return Time(d.count());
  }

 private:
  TimePoint tp_{};
};

class ClockInterface {
 public:
  virtual ~ClockInterface() = default;
  virtual std::unique_ptr<TimeInterface> Now() const = 0;
  virtual Time GetTime() const = 0;
};

class HighResolutionClock : public ClockInterface {
 public:
  std::unique_ptr<TimeInterface> Now() const override {
    return std::make_unique<ChronoTime>(
      std::chrono::high_resolution_clock::now());
  }
  Time GetTime() const override {
    return Now()->GetTime();
  }
};

}  // namespace core::utils

#endif  // CORE_UTILS_CLOCK_HPP_