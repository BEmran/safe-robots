// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_NAVIO_INCLUDE_NAVIO_HARDWARE_UTILS_HPP_
#define HARDWARE_NAVIO_INCLUDE_NAVIO_HARDWARE_UTILS_HPP_

#include <stdint.h>
#include <unistd.h>

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])

namespace navio::hardware_utils {
/**
 * @brief Sleep for some milliseconds, calls linux usleep function
 *
 * @param msec milli seconds to put thread in sleep
 */
void Delay(uint32_t msec);

constexpr const char* MPU_SPI_PATH = "/dev/spidev0.1";
constexpr auto NAVIO = 1;
constexpr auto NAVIO2 = 3;

int WriteFile(const char* path, const char* fmt, ...);

int ReadFile(const char* path, const char* fmt, ...);

bool CheckApm();

int GetNavioVersion();

}  // namespace navio::hardware_utils
#endif  // HARDWARE_NAVIO_INCLUDE_NAVIO_HARDWARE_UTILS_HPP_
