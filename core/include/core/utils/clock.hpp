// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_CLOCK_HPP_
#define CORE_UTILS_CLOCK_HPP_

#include <chrono>
#include <memory>

namespace core::utils {

class TimeStruct {
 public:
  TimeStruct() : secs{0}, micros{0}, in_sec{0.0} {
  }

  TimeStruct(long time_in_micro)
    : TimeStruct(static_cast<double>(time_in_micro) * 1e-6) {
  }

  TimeStruct(double time_in_sec) : in_sec(time_in_sec) {
    secs = static_cast<long>(in_sec);
    micros = static_cast<long>((in_sec - secs) * 1e6);
  }

  long Secs() {
    return secs;
  }

  long Micros() {
    return micros;
  }

  double InSec() {
    return in_sec;
  }

  std::chrono::duration<double> ToChronoDuration() {
    return std::chrono::duration<double, std::ratio<1>>{in_sec};
  }

 private:
  long secs{0};
  long micros{0};
  double in_sec{0.0};
};

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