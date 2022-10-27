// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_CLOCK_HPP_
#define CORE_UTILS_CLOCK_HPP_

#include <chrono>
#include <iostream>
#include <memory>

namespace core::utils {

namespace {
constexpr long MEGA = 1e6;
constexpr double MICRO = 1e-6;

constexpr double ToSec(const long micro) {
  return static_cast<double>(micro) * MICRO;
}

constexpr long ToMicro(const double sec) {
  return static_cast<long>(sec * MEGA);
}
}  // namespace

class TimeStruct {
 public:
  TimeStruct() : secs{0}, micros{0}, in_sec{0.0} {
  }

  TimeStruct(long time_in_micro) : TimeStruct(ToSec(time_in_micro)) {
  }

  TimeStruct(double time_in_sec) : in_sec(time_in_sec) {
    secs = static_cast<long>(in_sec);
    micros = ToMicro(in_sec - static_cast<double>(secs));
  }

  inline long Secs() const {
    return secs;
  }

  inline long Micros() const {
    return micros;
  }

  inline double InSec() const {
    return in_sec;
  }

  std::chrono::duration<double> ToChronoDuration() const {
    return std::chrono::duration<double, std::ratio<1>>{in_sec};
  }

  inline operator double() const {
    return in_sec;
  }

  inline std::string ToString() const {
    return std::to_string(in_sec);
  }

  inline std::string Pretty() const {
    return ToString() + " [sec]";
  }

 private:
  long secs{0};
  long micros{0};
  double in_sec{0.0};
};

std::ostream& operator<<(std::ostream& os, const TimeStruct& time) {
  return os << time.ToString();
}

class TimeInterface {
 public:
  virtual ~TimeInterface() = default;
  virtual TimeStruct Time() const = 0;
};

class SimpleTime : public TimeInterface {
 public:
  SimpleTime(TimeStruct time) : time_(time) {
  }

  TimeStruct Time() const override {
    return time_;
  }

 private:
  TimeStruct time_;
};

class ChronoTime : public TimeInterface {
  using TimePoint = std::chrono::system_clock::time_point;
  using Duration = std::chrono::duration<double>;

 public:
  ChronoTime(TimePoint tp) : tp_(tp) {
    Duration d = tp_.time_since_epoch();
    time_ = TimeStruct(d.count());
  }

  TimePoint GetTimePoint() const {
    return tp_;
  }

  TimeStruct Time() const override {
    return time_;
  }

 private:
  TimePoint tp_{};
  TimeStruct time_;
};

class ClockInterface {
 public:
  virtual ~ClockInterface() = default;
  virtual std::unique_ptr<TimeInterface> Now() = 0;
  virtual TimeStruct TimeNow() = 0;
};

class MockClock : public ClockInterface {
 public:
  std::unique_ptr<TimeInterface> Now() override {
    return std::make_unique<SimpleTime>(time_);
  }
  TimeStruct TimeNow() override {
    return Now()->Time();
  }
  void Set(TimeStruct time) {
    time_ = time;
  }

 private:
  TimeStruct time_;
};

class HighResolutionClock : public ClockInterface {
 public:
  std::unique_ptr<TimeInterface> Now() override {
    return std::make_unique<ChronoTime>(
      std::chrono::high_resolution_clock::now());
  }
  TimeStruct TimeNow() override {
    return Now()->Time();
  }
};

}  // namespace core::utils

#endif  // CORE_UTILS_CLOCK_HPP_