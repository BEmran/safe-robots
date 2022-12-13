// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef NAVIO_HARDWARE_UTILS_HPP_
#define NAVIO_HARDWARE_UTILS_HPP_

#include <unistd.h>

#include <string_view>

namespace hardware::navio::utils {

constexpr std::string_view MPU_SPI_PATH{"/dev/spidev0.1"};
constexpr std::string_view LSM_MAG_PATH{"/dev/spidev0.2"};
constexpr std::string_view LSM_A_G_PATH{"/dev/spidev0.3"};

constexpr uint8_t NAVIO = 1;
constexpr uint8_t NAVIO2 = 3;

bool CheckApm();

int GetNavioVersion();

}  // namespace hardware::navio::utils
#endif  // NAVIO_HARDWARE_UTILS_HPP_
