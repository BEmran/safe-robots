// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_NAVIO_SENSORS_MPU_MPU9250_HPP_
#define HARDWARE_NAVIO_SENSORS_MPU_MPU9250_HPP_

#include <memory>
#include <string_view>

#include "common/sensors/mpu/def.hpp"
#include "common/sensors/mpu/mpu9250.hpp"
#include "core/utils/node.hpp"

namespace hardware::navio::sensors::mpu {
using Mpu9250 = hardware::common::sensors::mpu::Mpu9250;
using Config = hardware::common::sensors::mpu::Config;

constexpr std::string_view SPI_PATH{"/dev/spidev0.1"};

class NavioMpu9250 : public Mpu9250 {
 public:
  NavioMpu9250(const Config& config, std::unique_ptr<core::utils::Node> node);
};
}  // namespace hardware::navio::sensors::mpu
#endif  // HARDWARE_NAVIO_SENSORS_MPU_MPU9250_HPP_
