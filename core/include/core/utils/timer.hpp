// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_TIMER_HPP_
#define CORE_UTILS_TIMER_HPP_

#include <memory>

#include "core/utils/clock.hpp"
#include "core/utils/time.hpp"

namespace core::utils {

/**
 * @brief Class with internal memory. Uses pulling method to check if the set
 * duration time is up.
 *
 */
class Timer {
 public:
  /**
   * @brief Construct a new Timer object with specific duration and default
   * clock source
   *
   * @param duration time duration
   */
  explicit Timer(const Time& duration);

  /**
   * @brief Construct a new Timer object
   *
   * @param duration time duration
   * @param clock ptr to clock source
   */
  Timer(const Time& duration, std::shared_ptr<ClockSource> clock);

  /**
   * @brief set new duration
   *
   * @param duration new duration
   */
  void SetDuration(const Time& duration);

  /**
   * @brief rest internal time
   *
   */
  void ResetTime();

  /**
   * @brief Check if time is up and reset time for next check if true
   *
   * @return true time is up
   * @return false otherwise
   */
  bool IsTime();

 private:
  /// @brief time duration in sec
  double duration_;

  /// @brief ptr to internal clock object
  std::shared_ptr<ClockSource> clock_;

  ///@brief previous running time
  Time ptime_;
};

}  // namespace core::utils

#endif  // CORE_UTILS_TIMER_HPP_