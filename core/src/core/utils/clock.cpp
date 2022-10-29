// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/clock.hpp"

namespace core::utils {

namespace {
// using Duration = std::chrono::duration<double, std::ratio<1>>;
constexpr long MEGA = 1e6;
constexpr double MICRO = 1e-6;
}  // namespace

TimeComponent::TimeComponent() : seconds{0}, micros{0} {
}

TimeComponent::TimeComponent(const long sec, const long micro)
  : seconds{sec}, micros{micro} {
}

TimeComponent::TimeComponent(const double time_in_sec) {
  seconds = static_cast<long>(time_in_sec);
  micros = static_cast<long>(time_in_sec * MEGA) - seconds * MEGA;
}

double TimeComponent::ToSeconds() const {
  return static_cast<double>(seconds) + static_cast<double>(micros) * MICRO;
}

Time::Time() : in_sec_{0.0} {
}

Time::Time(const double time_in_sec) : in_sec_(time_in_sec) {
}

Time::Time(const TimeComponent& tc) : in_sec_(tc.ToSeconds()) {
}

TimeComponent Time::Component() const {
  return TimeComponent(in_sec_);
}

double Time::InSeconds() const {
  return in_sec_;
}

uint64_t Time::InMicroSeconds() const {
  return static_cast<uint64_t>(in_sec_ * MEGA);
}

Duration Time::ToChronoDuration() const {
  return Duration{in_sec_};
}

Time::operator double() const {
  return in_sec_;
}

std::string Time::ToString() const {
  return std::to_string(in_sec_);
}

std::string Time::Pretty() const {
  return ToString() + " [sec]";
}

std::ostream& operator<<(std::ostream& os, const Time& time) {
  return os << time.ToString();
}

Time HighResolutionClock::Now() const {
  const Duration d = TimePoint().time_since_epoch();
  return Time{d.count()};
}

HighResolutionClock::ChronoTimePoint HighResolutionClock::TimePoint() {
  return std::chrono::high_resolution_clock::now();
}

double TimeInSeconds(std::unique_ptr<ClockInterface> clock) {
  const Time now = clock->Now();
  return now.InSeconds();
}

uint64_t TimeInMicroSeconds(std::unique_ptr<ClockInterface> clock) {
  const Time now = clock->Now();
  return now.InMicroSeconds();
}

std::string TimeInSecondsString(std::unique_ptr<ClockInterface> clock) {
  const Time now = clock->Now();
  return now.ToString();
}

}  // namespace core::utils
