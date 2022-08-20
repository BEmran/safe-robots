#include "mpu/mpu9250.hpp"
#include "navio/util.h"
#include <unistd.h>
#include <string>
#include <memory>

constexpr mpu::AccelScale ASCALE = mpu::AccelScale::FS_4G;
constexpr mpu::AccelBandWidthHz ABW = mpu::AccelBandWidthHz::BW_21HZ;
constexpr mpu::GyroScale GSCALE = mpu::GyroScale::FS_250DPS;
constexpr mpu::GyroBandWidthHz GBW = mpu::GyroBandWidthHz::BW_250HZ;
constexpr mpu::MagMode MMODE = mpu::MagMode::CONTINUES_100HZ;
constexpr mpu::MagScale MSCALE = mpu::MagScale::FS_16BITS;
constexpr uint8_t SAMPLE_RATE_DIVISOR = 4;

//=============================================================================
int main(int /*argc*/, char** /*argv[]*/)
{
  if (navio::CheckApm())
  {
    return 1;
  }
  mpu::Config config;
  config.accel_bw = ABW;
  config.accel_scale = ASCALE;
  config.gyro_bw = GBW;
  config.gyro_scale = GSCALE;
  config.mag_mode = MMODE;
  config.mag_scale = MSCALE;
  config.sample_rate_divisor = SAMPLE_RATE_DIVISOR;
  auto spi = std::make_unique<SPI>(navio::MPU_SPI_PATH, true);
  auto sensor = std::make_unique<mpu::Mpu9250>(config, std::move(spi), true);
    
  if (!sensor->Probe())
  {
    return EXIT_FAILURE;
  }
  sensor->Initialize();
  
  //-------------------------------------------------------------------------

  while (true)
  {
    sensor->Update();
    std::cout << sensor->GetData();
    usleep(500000);
  }
  return EXIT_SUCCESS;
}