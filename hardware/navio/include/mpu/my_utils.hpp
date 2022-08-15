#ifndef MPU_MY_UTILS_HPP
#define MPU_MY_UTILS_HPP

#include <stdint.h>
#include <unistd.h>
#include "mpu/spi.hpp"
#include "mpu/mpu9250_register_map.h"
#include <core/utils/math.hpp>

namespace mpu
{

constexpr int max_bit_val = 32767;  // 2^16/2

inline constexpr uint8_t operator"" _uc(unsigned long long arg) noexcept
{
  return static_cast<uint8_t>(arg);
}

using core::utils::GRAVITY;
using core::utils::PI;

spi::SPI* GetSpi();

void Delay(uint32_t msec);

// Turn the MSB and LSB into a signed 16-bit value
int16_t To16Bit(const uint8_t msb, const uint8_t lsb);

}  // namespace mpu

#endif  // MPU_MY_UTILS_HPP