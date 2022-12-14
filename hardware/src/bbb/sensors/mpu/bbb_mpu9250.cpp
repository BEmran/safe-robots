// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "bbb/sensors/mpu/bbb_mpu9250.hpp"

#include "bbb/utils.hpp"
#include "common/comm/i2c.hpp"
#include "core/utils/node.hpp"

namespace hardware::bbb::sensors::mpu {
using I2C = hardware::common::comm::I2C;

std::unique_ptr<I2C> CreateI2c() {
  return std::make_unique<I2C>(I2C_MPU_BUS, I2C_MPU_ADDRESS);
}

std::unique_ptr<core::utils::Node> CreateNode(std::string_view name) {
  return std::make_unique<core::utils::Node>(
    core::utils::CreateNodeUsingSystemLogger(name));
}

BbbMpu9250::BbbMpu9250(const Config& config,
                       std::unique_ptr<core::utils::Node> node)
  : Mpu9250(config, CreateI2c(), std::move(node)) {
}

BbbMpu9250::BbbMpu9250(const Config& config, std::string_view name)
  : Mpu9250(config, CreateI2c(), CreateNode(name)) {
}
}  // namespace hardware::bbb::sensors::mpu