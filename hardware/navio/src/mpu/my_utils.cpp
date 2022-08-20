#include "mpu/my_utils.hpp"
#include <iostream>

#include <algorithm>

namespace mpu
{

spi::SPI* GetSpi()
{
  static spi::SPI spi("/dev/spidev0.1", true);
  return &spi;
}

void Delay(uint32_t msec)
{
  usleep(msec * 1000);
}

int16_t To16Bit(const uint8_t msb, const uint8_t lsb)
{
  const auto high = static_cast<int>(msb);
  const auto low = static_cast<int>(lsb);
  return static_cast<int16_t>(high << 8 | low);
}

SensorSpecs CreateSensorSpecs(const MATH_TYPE scale, const MATH_TYPE unit)
{
  const auto sen = max_bit_val / scale;
  std::cout << "scale: " << scale << ", sensitivity: " << sen
            << " unit:" << unit << std::endl;
  return SensorSpecs(sen, unit);
}

core::utils::Vec3 ArrayToVec3(const std::array<MATH_TYPE, 3>& array)
{
  return core::utils::Vec3{array[0], array[1], array[2]};
}

std::array<MATH_TYPE, 3> ApplySensorSpecs(const std::array<int16_t, 3>& raw,
                                          const SensorSpecs& spec)
{
  std::array<MATH_TYPE, 3> data;
  std::transform(raw.begin(), raw.end(), data.begin(), [&spec](const auto r) {
    return spec.Apply(static_cast<MATH_TYPE>(r));
  });
  return data;
}
}  // namespace mpu
