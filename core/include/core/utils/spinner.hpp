// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_SPINNER_HPP_
#define CORE_UTILS_SPINNER_HPP_

#include <chrono>
#include <memory>

#include "core/utils/clock.hpp"
#include "core/utils/logger_node.hpp"

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
  Spinner(const double hz, std::shared_ptr<ClockInterface> clock,
          std::shared_ptr<NodeLogger> logger);

  /**
   * @brief Spin the time once
   * @details a blocking function sleeps until calculated time is passed
   *
   * @return double actual sampling time (dt)
   */
  double SpinOnce();

  void SetRate(const double hz);

  void SetSamplingTime(const double st_sec);

 protected:
  /**
   * @brief prints spinning information on screen for debugging purposes
   *
   * @param dt actual sampling time to print
   */
  void PrintInfo(const double dt);

  // update max min data if not the first time
  void update_max_min_values(const double dt);

  double UpdateTime();

 private:
  std::unique_ptr<ClockInterface> clock_;
  std::shared_ptr<NodeLogger> logger_;
  Time sampling_time_{1.};  ///> sampling time, 1/rate
  double min_dt_{1e9};      ///> minimum sampling time
  double max_dt_{0.};       ///> minimum sampling time
  Time ptime_before_sleep;  ///> previous spin's time before sleep
  Time ptime_after_sleep;   ///> previous running time
  Time ptime_debug;         ///> previous time for printing debugging msg
};

}  // namespace core::utils

#endif  // CORE_UTILS_SPINNER_HPP_