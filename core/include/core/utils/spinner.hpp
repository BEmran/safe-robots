// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_SPINNER_HPP_
#define CORE_UTILS_SPINNER_HPP_

#include <chrono>
#include <memory>

#include "core/utils/clock.hpp"
#include "core/utils/logger_node.hpp"
#include "core/utils/min_max_statistics.hpp"
#include "core/utils/time.hpp"
#include "core/utils/timer.hpp"

namespace core::utils {
/**
 * @brief A blocking class used to ensure a loop will take a certain frequency
 * time
 *
 */
class Spinner {
  using TimePoint = std::chrono::system_clock::time_point;
  using Duration = std::chrono::duration<double>;

 public:
  /**
   * @brief Construct a new Spinner object
   *
   * @param hz rate of spinning in hz
   */
  Spinner(const double hz);

  /**
   * @brief Construct a new Spinner object
   *
   * @param hz rate of spinning in hz
   */
  Spinner(const double hz, std::shared_ptr<ClockSource> clock,
          std::shared_ptr<NodeLogger> logger);

  /**
   * @brief Spin the time once
   * @details a blocking function sleeps until calculated time is passed
   *
   * @return double actual sampling time (dt)
   */
  double SpinOnce();

  /**
   * @brief Set object's Rate
   * @details it reset the max and min information
   *
   * @param hz rate in hertz
   */
  void SetRate(const double hz);

  /**
   * @brief Set object's Sampling Time
   * @details it reset the max and min information
   *
   * @param st_sec sampling time in seconds
   */
  void SetSamplingTime(const double st_sec);

 protected:
  double CalculateActualSamplingTime(const Time& ctime);

  /**
   * @brief Calculate if sleep is needed to meet the set rate
   *
   * @param ctime current time
   */
  void SleepIfNeeded(const Time& ctime);

  /**
   * @brief prints spinning information on screen for debugging purposes
   *
   */
  void PrintInfoIfTimer();

  void UpdateStatistics(const double actual_sampling_time);

 private:
  /// @brief ptr to internal clock object
  std::shared_ptr<ClockSource> clock_;
  /// @brief ptr to logger object
  std::shared_ptr<NodeLogger> logger_;
  /// @brief sampling time, 1/rate
  Time sampling_time_{1.};
  /// @brief previous spin time used to calculate actual sampling time
  Time ptime_before_sleep_;
  /// @brief previous running time needed as sleep_for function is not accurate
  Time ptime_;
  /// @brief used to store and print statistics information
  MinMaxStatistics<double> statics_;
  /// @brief timer for debugging statistics information
  Timer debug_timer_;
  bool first_statistics_{true};
};

}  // namespace core::utils

#endif  // CORE_UTILS_SPINNER_HPP_