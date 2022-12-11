// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef BBB_UTILS_HPP_
#define BBB_UTILS_HPP_

#include <algorithm>  // copy
#include <array>
#include <cmath>
#include <cstdint>  // for uint8_t types etc
#include <optional>
#include <vector>

// constant numbers
constexpr double PI = M_PI;
constexpr double TWO_PI = 2 * M_PI;
/// @brief multiply to convert degrees to radians
constexpr double DEG_TO_RAD = 0.0174532925199;
/// @brief multiply to convert radians to degrees
constexpr double RAD_TO_DEG = 57.295779513;
/// @brief multiply to convert radians to degrees
constexpr double MS2_TO_G = 0.10197162129;
/// @brief multiply to convert G to m/s^2, standard gravity definition
constexpr double G_TO_MS2 = 9.80665;

/**
 * @brief describes the order in which a sequence of bytes is stored in computer
 * memory.
 */
enum class EndianByteOrder {
  BIG,    // the most significant value is stored first at the lowest
          // storage address.
  LITTLE  // the least significant value is stored first at the
          // lowest storage address.
};

int16_t RegisterBytesToWord(const uint8_t reg0, const uint8_t reg1,
                            const EndianByteOrder order);

std::vector<int16_t> RegisterBytesToWords(const std::vector<uint8_t> bytes,
                                          const EndianByteOrder order);

std::pair<uint8_t, uint8_t> WordToRegisterBytes(const int16_t word,
                                                const EndianByteOrder order);

std::vector<uint8_t> WordsToRegisterBytes(const std::vector<int16_t> words,
                                          const EndianByteOrder order);

/**
 * @brief sleep the process for the passed time in  milliseconds
 *
 * @param milli duration to sleep for in milliseconds
 */
void MilliSleep(const size_t milli);

/**
 * @brief sleep the process for the passed time in microseconds
 *
 * @param micro duration to sleep for in microseconds
 */
void MicroSleep(const size_t micro);

template <typename T, size_t SIZE>
std::array<T, SIZE> CopyVectorToArray(const std::vector<T>& vec) {
  std::array<int16_t, 3> arr;
  std::copy(vec.begin(), vec.end(), arr.begin());
  return arr;
}

#endif  // BBB_UTILS_HPP_
