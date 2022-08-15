#include "mpu/my_utils.hpp"

namespace mpu {

spi::SPI* GetSpi()
{
  static spi::SPI spi("/dev/spidev0.1", false);
  return &spi;
}

void Delay(uint32_t msec)
{
  usleep(msec * 1000);
}

int16_t To16Bit(const uint8_t msb, const uint8_t lsb)
{
  return (static_cast<int16_t>(msb) << 8) | static_cast<int16_t>(lsb);
}
} // namespace mpu 
