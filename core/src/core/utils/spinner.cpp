// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/spinner.hpp"

#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>

/// @brief period to print information in sec
const std::chrono::duration<double> DEBUG_TIME_IN_SECOND(2);

namespace spinner {

Spinner::Spinner(const double hz, const bool debug) : debug_(debug) {
  SetRate(hz);
  ptime_after_sleep = std::chrono::high_resolution_clock::now();
}

// Spinner::Spinner(const double sampling_time_sec, const bool debug)
//   : debug_(debug) {
//   SetSamplingTime(sampling_time_sec);
// }

void Spinner::SetRate(const double hz) {
  if (hz <= 0) {
    return;
  }
  sampling_time_ = Duration(1 / hz);
}

void Spinner::SetSamplingTime(const double st_sec) {
  if (st_sec <= 0) {
    std::cerr << "Failed to set sampling time:"
              << " expected value to be bigger than 0 and got " << st_sec
              << " Current sampling time value unchanged = "
              << sampling_time_.count() << std::endl;
    return;
  }
  sampling_time_ = Duration(st_sec);
}

double Spinner::UpdateTime() {
  auto ctime = std::chrono::high_resolution_clock::now();
  // check time difference in milli
  Duration dt = ctime - ptime_after_sleep;
  // ptime_ = ctime;
  return dt.count();
}

double Spinner::SpinOnce() {
  const auto ctime = std::chrono::high_resolution_clock::now();
  Duration actual_sampling_time = ctime - ptime_before_sleep;
  Duration elapsed = ctime - ptime_after_sleep;
  if (sampling_time_ > elapsed) {
    std::this_thread::sleep_for(sampling_time_ - elapsed);
  } else {
    std::cerr << "violate sleeping time: Required Sampling time is too small: "
              << sampling_time_.count()
              << " should be at least: " << elapsed.count() << std::endl;
  }
  ptime_before_sleep = ctime;
  ptime_after_sleep = std::chrono::high_resolution_clock::now();

  update_max_min_values(actual_sampling_time.count());
  if (debug_) {
    PrintInfo(actual_sampling_time.count());
  }
  return actual_sampling_time.count();
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
  static double sum{0};
  counter++;
  sum += dt;
  if (sum > 2) {
    double avg = sum / counter;
    printf(
      "[Spinner][%4.2f hz]: loop is running average: %4.2fHz, max: "
      "%4.2fHz, min %4.2fHz\n",
      1 / sampling_time_.count(), 1.0 / avg, 1.0 / min_dt_, 1.0 / max_dt_);
    counter = 0;
    sum = 0;
  }
}
}  // namespace spinner