// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_CLOCK_HPP_
#define CORE_UTILS_CLOCK_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "core/utils/time.hpp"

namespace core::utils {

/**
 * @brief Interface to implement different method to capture time
 *
 */
class ClockSource {
 public:
  /**
   * @brief Destroy the Clock Interface object
   *
   */
  virtual ~ClockSource() = default;

  /**
   * @brief Returns current time
   *
   * @return Time current time
   */
  virtual Time Now() const = 0;
};

/**
 * @brief Concrete implementation to ClockSource uses
 * chrono::high_resolution_clock as its internal clock implementation.
 *
 */
class HighResolutionClock : public ClockSource {
  using ChronoTimePoint = std::chrono::system_clock::time_point;

 public:
  /**
   * @implements Now function from ClockSource
   */
  Time Now() const override;

  /**
   * @brief Returns current time as time_point
   *
   * @return ChronoTimePoint current time point
   */
  static ChronoTimePoint TimePoint();
};

/**
 * @brief Create a shared ptr to default clock source
 *
 * @return std::shared_ptr<ClockSource> ptr to ClockSource
 */
std::shared_ptr<ClockSource> DefaultClockSource();

/**
 * @brief Returns current time in seconds using the passed clock.
 * @details It simplifies the procedure of calculating current time in seconds
 * using the DefaultClockSource as it is default clock source.
 *
 * @param clock clock used to calculate time
 * @return double current time in seconds
 */
double TimeInSeconds(std::shared_ptr<ClockSource> clock = DefaultClockSource());
/**
 * @brief Returns current time in microseconds using the passed clock.
 * @details It simplifies the procedure of calculating current time in
 * microseconds using the DefaultClockSource as it is default clock
 * source.
 *
 * @param clock clock used to calculate time
 * @return uint64_t current time in microseconds
 */
uint64_t
TimeInMicroSeconds(std::shared_ptr<ClockSource> clock = DefaultClockSource());

/**
 * @brief Returns a string contains the current time in seconds using the passed
 * clock.
 * @details It simplifies the procedure of creating a string contains the
 * current time in seconds using the DefaultClockSource as it is default clock
 * source.
 *
 * @param clock clock used to calculate time
 * @return std::string time in seconds as string
 */
std::string
TimeInSecondsString(std::shared_ptr<ClockSource> clock = DefaultClockSource());

}  // namespace core::utils

#endif  // CORE_UTILS_CLOCK_HPP_