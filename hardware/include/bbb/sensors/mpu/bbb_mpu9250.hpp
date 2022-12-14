// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_BBB_SENSORS_MPU_MPU9250_HPP_
#define HARDWARE_BBB_SENSORS_MPU_MPU9250_HPP_

#include <memory>
#include <string_view>

#include "common/sensors/mpu/def.hpp"
#include "common/sensors/mpu/mpu9250.hpp"
#include "core/utils/node.hpp"

namespace hardware::bbb::sensors::mpu {
using Mpu9250 = hardware::common::sensors::mpu::Mpu9250;
using Config = hardware::common::sensors::mpu::Config;

constexpr uint8_t I2C_MPU_BUS = 0x02;
constexpr uint8_t I2C_MPU_ADDRESS = common::sensors::mpu::mpu9250::I2C_ADDRESS1;

class BbbMpu9250 : public Mpu9250 {
 public:
  BbbMpu9250(const Config& config, std::unique_ptr<core::utils::Node> node);
  BbbMpu9250(const Config& config, std::string_view name = "mpu");
};
}  // namespace hardware::bbb::sensors::mpu
#endif  // HARDWARE_BBB_SENSORS_MPU_MPU9250_HPP_
