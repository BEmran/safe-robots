// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/spinner.hpp"

#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>

/// @brief period to print information in sec
const double DEBUG_TIME_IN_SECOND{2};
constexpr std::string_view SPINNER_NAME = "SPINNER";

namespace core::utils {

Spinner::Spinner(const double hz)
  : Spinner(hz, std::make_unique<HighResolutionClock>(),
            std::make_shared<NodeLogger>(
              CreateNodeLoggerUsingSystemLogger(SPINNER_NAME))) {
}

Spinner::Spinner(const double hz, std::unique_ptr<ClockInterface> clock,
                 std::shared_ptr<NodeLogger> logger)
  : clock_(std::move(clock)), logger_(logger) {
  SetRate(hz);
  ptime_after_sleep = clock_->Now();
  ptime_before_sleep = clock_->Now();
}

void Spinner::SetRate(const double hz) {
  if (hz <= 0) {
    return;
  }
  sampling_time_ = Time(1 / hz);
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
}

double Spinner::UpdateTime() {
  // auto ctime = std::chrono::high_resolution_clock::now();
  // check time difference in milli
  // Time dt = ctime - ptime_after_sleep;
  // ptime_ = ctime;
  return 1;
}

double Spinner::SpinOnce() {
  const auto ctime = clock_->Now();
  const Time actual_sampling_time = ctime - ptime_before_sleep;
  const Time elapsed = ctime - ptime_after_sleep;
  if (sampling_time_ > elapsed) {
    const Time sleep = sampling_time_ - elapsed;
    std::this_thread::sleep_for(ToChronoDuration(sleep));
  } else {
    logger_->Warn() << "violate sleeping time: Required Sampling time is too "
                       "small: "
                    << sampling_time_ << " should be at least: " << elapsed
                    << std::endl;
  }
  ptime_before_sleep = ctime;
  ptime_after_sleep = clock_->Now();

  update_max_min_values(actual_sampling_time);
  PrintInfo(actual_sampling_time);

  return actual_sampling_time;
}

void Spinner::update_max_min_values(const double dt) {
  static bool first = true;
  if (!first) {
    if (dt > max_dt_) {
      max_dt_ = dt;
    }
    if (dt < min_dt_) {
      min_dt_ = dt;
    }
  }
  first = false;
}

void Spinner::PrintInfo(const double dt) {
  static int counter{0};
  static double sum_sampling_time{0};
  counter++;
  sum_sampling_time += dt;
  if (sum_sampling_time > DEBUG_TIME_IN_SECOND) {
    const double avg_rate = counter / sum_sampling_time;
    const double spinner_rate = 1 / sampling_time_;
    logger_->Debug() << "[" << spinner_rate
                     << "Hz]: loop is running average: " << avg_rate
                     << "Hz, max: " << 1.0 / min_dt_ << "Hz, min "
                     << 1.0 / max_dt_ << "Hz";
    counter = 0;
    sum_sampling_time = 0;
  }
}
}  // namespace core::utils