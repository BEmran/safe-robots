// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_COMMON_UTILS_HPP_
#define HARDWARE_COMMON_UTILS_HPP_

#include <unistd.h>

#include <cstdint>

namespace hardware::common {
/**
 * @brief Sleep for some milliseconds, calls linux usleep function
 *
 * @param msec milli seconds to put thread in sleep
 */
void MilliDelay(const uint32_t msec);

/**
 * @brief Sleep for some microseconds, calls linux usleep function
 *
 * @param usec micro seconds to put thread in sleep
 */
void MicroDelay(const uint32_t usec);

int WriteFile(const char* path, const char* fmt, ...);

int ReadFile(const char* path, const char* fmt, ...);

template <typename T>
inline auto ToByte(const T t) {
  return static_cast<uint8_t>(t);
}
}  // namespace hardware::common
#endif  // HARDWARE_COMMON_UTILS_HPP_
