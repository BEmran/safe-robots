#ifndef MPU_MY_MPU_HPP
#define MPU_MY_MPU_HPP

#include <stdint.h>
#include <unistd.h>
#include "mpu/spi.hpp"
#include "mpu/mpu9250_register_map.h"

namespace mpu {

spi::SPI* GetSpi();

void delay(uint32_t msec);

// Turn the MSB and LSB into a signed 16-bit value
int16_t To16Bit(const uint8_t msb, const uint8_t lsb)
{
  return (static_cast<int16_t>(msb) << 8) | static_cast<int16_t>(lsb);
}
}

#endif  // MPU_MY_MPU_HPP