// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "navio/sensors/mpu/navio_mpu9250.hpp"

#include "common/comm/spi.hpp"

namespace hardware::navio::sensors::mpu {
using SPI = hardware::common::comm::SPI;

std::unique_ptr<SPI> CreateSpi() {
  return std::make_unique<SPI>(SPI_PATH);
}

NavioMpu9250::NavioMpu9250(const Config& config,
                           std::unique_ptr<core::utils::Node> node)
  : Mpu9250(config, CreateSpi(), std::move(node)) {
}

}  // namespace hardware::navio::sensors::mpu