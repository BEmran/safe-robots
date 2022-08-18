#include "mpu/mpu9250.hpp"
#include "navio/util.h"
#include <unistd.h>
#include <string>
#include <memory>

constexpr mpu::AccelScale ASCALE = mpu::AccelScale::AFS_16G;
constexpr mpu::GyroScale GSCALE = mpu::GyroScale::GFS_2000DPS;
constexpr mpu::MagScale MSCALE = mpu::MagScale::MFS_16BITS;
constexpr mpu::MagMode MMODE = mpu::MagMode::CONTINUES_100HZ_MODE;
constexpr uint8_t SAMPLE_RATE_DIVISOR = 4;

//=============================================================================
int main(int /*argc*/, char** /*argv[]*/)
{
  if (CheckApm())
  {
    return 1;
  }
  mpu::Config config;
  config.accel_scale = ASCALE;
  config.gyro_scale = GSCALE;
  config.mag_scale = MSCALE;
  config.mag_mode = MMODE;
  config.sample_rate_divisor = SAMPLE_RATE_DIVISOR;
  
  auto sensor = std::make_unique<mpu::Mpu9250>(config, true);
  
  sensor->fake();
  
  // sensor->Reset();

  // if (!sensor->Probe())
  // {
  //   return EXIT_FAILURE;
  // }

  // sensor->Initialize();
  
  //-------------------------------------------------------------------------

  while (true)
  {
    sensor->Update();
    std::cout << sensor->GetData();
    usleep(500000);
  }
  return EXIT_SUCCESS;
}