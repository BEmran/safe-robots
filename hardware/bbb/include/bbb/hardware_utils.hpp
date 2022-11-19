// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef BBB_HARDWARE_UTILS_HPP_
#define BBB_HARDWARE_UTILS_HPP_

#include <unistd.h>

#include <cstdint>

#define CALIBRATION_DIR "/var/lib/robotcontrol/"

#define ARRAY_SIZE(a) sizeof(a) / sizeof(a[0])  // NOLINT

namespace bbb::hardware_utils {
/**
 * @brief Sleep for some milliseconds, calls linux usleep function
 *
 * @param msec milli seconds to put thread in sleep
 */
void Delay(uint32_t msec);

constexpr const char* MPU_SPI_PATH = "/dev/spidev0.1";
constexpr const char* LSM_MAG_PATH = "/dev/spidev0.2";
constexpr const char* LSM_A_G_PATH = "/dev/spidev0.3";

constexpr auto BBB = 1;
constexpr auto BBB2 = 3;

int WriteFile(const char* path, const char* fmt, ...);

int ReadFile(const char* path, const char* fmt, ...);

bool CheckApm();

int GetBBBVersion();

}  // namespace bbb::hardware_utils
#endif  // BBB_HARDWARE_UTILS_HPP_
