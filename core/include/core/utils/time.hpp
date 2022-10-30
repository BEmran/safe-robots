// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_TIME_HPP_
#define CORE_UTILS_TIME_HPP_

#include <chrono>
#include <iostream>

namespace core::utils {

/**
 * @brief Hold time detail information
 * @details time is represented as [seconds.micros]
 */
struct TimeComponent {
  /// @brief number of seconds in a time
  long seconds{0};
  /// @brief number of microseconds in a time
  long micros{0};

  /**
   * @brief Default Construct of a new Time Component object
   *
   */
  TimeComponent();

  /**
   * @brief Construct a new Time Component object using detail information
   *
   * @param sec number of seconds in a time
   * @param micro number of microseconds in a time
   */
  TimeComponent(const long sec, const long micro);
  /**
   * @brief Construct a new Time Component object using a time represented in
   * seconds
   *
   * @param time_in_sec a time in seconds
   */
  explicit TimeComponent(const double time_in_sec);

  /**
   * @brief construct a full time in seconds from the component
   *
   * @return double time in seconds
   */
  double ToSeconds() const;
};

/**
 * @brief A class contains time information used to
 *
 */
class Time {
  /// @brief duration in double and a ration of 1
  using Duration = std::chrono::duration<double, std::ratio<1>>;

 public:
  /**
   * @brief Default construct of a new Time object
   *
   */
  Time();

  /**
   * @brief Construct a new Time object using the passed time
   *
   * @param time_in_sec time in seconds
   */
  Time(const double time_in_sec);

  /**
   * @brief Construct a new Time object using TimeComponent object
   *
   * @param tc  object holds time detail information
   */
  explicit Time(const TimeComponent& tc);

  /**
   * @brief Convert Time to TimeComponent object to get detail information
   *
   * @return TimeComponent object holds detail information
   */
  TimeComponent Component() const;

  /**
   * @brief time in seconds
   *
   * @return double time in seconds
   */
  double InSeconds() const;

  /**
   * @brief convert time to microseconds
   *
   * @return uint64_t time in microseconds
   */
  uint64_t InMicroSeconds() const;

  /**
   * @brief implicit conversion to represent Time object as double number to
   * simplify the arithmetic operation
   *
   * @return double time in seconds
   */
  operator double() const;

  /**
   * @brief convert the time in seconds to a string
   *
   * @return std::string a string contains time
   */
  std::string ToString() const;

  /**
   * @brief a pretty method to display the time with the SIU unit
   *
   * @return std::string a string contains time and unit
   */
  std::string Pretty() const;

 private:
  /// @brief time in seconds
  double in_sec_{0.0};
};

/**
 * @brief Steam method to display Time object
 *
 * @param os a reference to output-stream object
 * @param time time object to be displayed
 * @return std::ostream& a reference to output-stream object  with the appended
 * time
 */
std::ostream& operator<<(std::ostream& os, const Time& time);

/**
 * @brief Convert time to Chrono Duration object
 *
 * @return Duration chrono::duration object of type double
 */
std::chrono::duration<double> ToChronoDuration(const Time& time);

}  // namespace core::utils

#endif  // CORE_UTILS_TIME_HPP_