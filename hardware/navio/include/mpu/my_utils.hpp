#ifndef MPU_MY_UTILS_HPP
#define MPU_MY_UTILS_HPP

#include <stdint.h>
#include <unistd.h>
#include "mpu/spi.hpp"
#include "mpu/mpu9250_register_map.h"
#include <core/utils/math.hpp>

namespace mpu {

using core::utils::GRAVITY;
using core::utils::PI;

spi::SPI* GetSpi();

void delay(uint32_t msec);

// Turn the MSB and LSB into a signed 16-bit value
int16_t To16Bit(const uint8_t msb, const uint8_t lsb);

}  // namespace mpu

#endif  // MPU_MY_UTILS_HPP