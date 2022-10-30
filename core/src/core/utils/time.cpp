// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/time.hpp"

namespace core::utils {

namespace {
/// @brief Mega scale used for conversion
constexpr long MEGA = 1e6;
/// @brief Micro scale used for conversion
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

std::chrono::duration<double> ToChronoDuration(const Time& time) {
  return std::chrono::duration<double>{time.InSeconds()};
}
}  // namespace core::utils
