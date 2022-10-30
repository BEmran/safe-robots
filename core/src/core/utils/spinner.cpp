// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/spinner.hpp"

#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>

namespace core::utils {
namespace {
/// @brief period to print information in sec
constexpr double DEBUG_TIME_IN_SECOND{2};

/// @brief default module name used during logging
constexpr std::string_view DEFAULT_LOGGING_NAME{"SPINNER"};
}  // namespace

Spinner::Spinner(const double hz)
  : Spinner(hz, DefaultClockSource(),
            std::make_shared<NodeLogger>(
              CreateNodeLoggerUsingSystemLogger(DEFAULT_LOGGING_NAME))) {
}

Spinner::Spinner(const double hz, std::shared_ptr<ClockSource> clock,
                 std::shared_ptr<NodeLogger> logger)
  : clock_(clock)
  , logger_(logger)
  , debug_timer_{Time(DEBUG_TIME_IN_SECOND), clock} {
  SetRate(hz);
  ptime_ = clock_->Now();
  ptime_before_sleep_ = ptime_;
}

void Spinner::SetRate(const double hz) {
  SetSamplingTime(1 / hz);
}

void Spinner::SetSamplingTime(const double st_sec) {
  if (st_sec <= 0) {
    logger_->Warn() << "Failed to set sampling time:"
                    << " expected value to be bigger than 0 and got " << st_sec
                    << " Current sampling time value unchanged = "
                    << sampling_time_ << std::endl;
    return;
  }
  sampling_time_ = Time(st_sec);
  statics_.ResetMinMax();
}

double Spinner::SpinOnce() {
  const auto ctime = clock_->Now();

  const double actual_sampling_time = CalculateActualSamplingTime(ctime);

  SleepIfNeeded(ctime);

  statics_.Update(actual_sampling_time);

  PrintInfoIfTimer();

  ptime_ = clock_->Now();
  return actual_sampling_time;
}

double Spinner::CalculateActualSamplingTime(const Time& ctime) {
  const double actual_sampling_time = ctime - ptime_before_sleep_;
  ptime_before_sleep_ = ctime;
  return actual_sampling_time;
}

void Spinner::SleepIfNeeded(const Time& ctime) {
  const Time elapsed = ctime - ptime_;
  const Time sleep = sampling_time_ - elapsed;

  if (sleep > 0) {
    std::this_thread::sleep_for(ToChronoDuration(sleep));

  } else {
    logger_->Warn() << "violate sleeping time: Required Sampling time is too "
                       "small: "
                    << sampling_time_ << " should be at least: " << elapsed
                    << std::endl;
  }
}

void Spinner::PrintInfoIfTimer() {
  if (debug_timer_.IsTime()) {
    logger_->Debug() << "[" << 1 / sampling_time_ << "Hz]: is running with "
                     << statics_;
  }
}
}  // namespace core::utils