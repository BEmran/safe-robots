// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_MIN_MAX_STATISTICS_HPP_
#define CORE_UTILS_MIN_MAX_STATISTICS_HPP_

#include <climits>
#include <ostream>

namespace core::utils {

template <typename T>
struct MinMaxStatistics {
  ///@brief minimum value
  T min_value{std::numeric_limits<T>::max()};

  ///@brief minimum value
  T max_value{std::numeric_limits<T>::min()};

  /**
   * @brief reset minimum and maximum values
   *
   */
  void ResetMinMax() {
    min_value = std::numeric_limits<T>::max();
    max_value = std::numeric_limits<T>::min();
  }

  T CalculateAverage() const {
    return max_value - min_value;
  }

  /**
   * @brief Update minimum and maximum values
   *
   * @param value new value
   */
  void Update(const T& value) {
    if (value < min_value) {
      min_value = value;
    }
    if (value > max_value) {
      max_value = value;
    }
  }
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const MinMaxStatistics<T>& stat) {
  return os << " average: " << stat.CalculateAverage()
            << " max: " << stat.max_value << " min: " << stat.min_value;
}

}  // namespace core::utils

#endif  // CORE_UTILS_MIN_MAX_STATISTICS_HPP_