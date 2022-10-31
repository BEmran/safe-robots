// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_MIN_MAX_STATISTICS_HPP_
#define CORE_UTILS_MIN_MAX_STATISTICS_HPP_

#include <limits>
#include <ostream>

namespace core::utils {

/**
 * @brief Calculate statistics of some value.
 *
 * @tparam T type of value must be integral type
 */
template <typename T>
struct MinMaxStatistics {
  ///@brief minimum value
  T min_value{std::numeric_limits<T>::max()};

  ///@brief minimum value
  T max_value{std::numeric_limits<T>::min()};

  ///@brief counter used to calculate average
  uint32_t counter{0};

  ///@brief summation to calculate average
  double sum{0.0};

  /**
   * @brief reset minimum and maximum values
   *
   */
  void ResetMinMax() {
    min_value = std::numeric_limits<T>::max();
    max_value = std::numeric_limits<T>::min();
    counter = 0;
    sum = 0.0;
  }

  double CalculateAverage() const {
    if (counter > 0) {
      return sum / counter;
    } else {
      return 0.0;
    }
  }

  /**
   * @brief Update minimum and maximum values
   *
   * @param value new value
   */
  void Update(const T& value) {
    sum += value;
    counter++;
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
  return os << "average: " << stat.CalculateAverage()
            << " min: " << stat.min_value << " max: " << stat.max_value;
}

}  // namespace core::utils

#endif  // CORE_UTILS_MIN_MAX_STATISTICS_HPP_