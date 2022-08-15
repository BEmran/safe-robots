#include "mpu/AK8963.hpp"
#include "navio/util.h"
#include <unistd.h>
#include <string>
#include <memory>

// static const MPUIMU::Ascale_t ASCALE = MPUIMU::AFS_16G;
// static const MPUIMU::Gscale_t GSCALE = MPUIMU::GFS_2000DPS;
static const mpu::MagScale MSCALE = mpu::MagScale::MFS_16BITS;
static const mpu::MagMode MMODE = mpu::MagMode::CONTINUES_100HZ_MODE;
// static const uint8_t SAMPLE_RATE_DIVISOR = 4;

//=============================================================================
int main(int /*argc*/, char** /*argv[]*/)
{
  if (CheckApm())
  {
    return 1;
  }
  mpu::AK8963Config config;
  config.mode = MMODE;
  config.scale = MSCALE;
  auto sensor = std::make_unique<mpu::AK8963>(config, true);

  // // Start the MPU9250
  // switch (sensor->begin())
  // {
  //   case MPUIMU::ERROR_IMU_ID:
  //     printf("Bad IMU device ID\n");
  //   case MPUIMU::ERROR_MAG_ID:
  //     printf("Bad magnetometer device ID\n");
  //   case MPUIMU::ERROR_SELFTEST:
  //     printf("Failed self-test\n");
  //     break;
  //     return EXIT_FAILURE;
  //   default:
  //     printf("MPU9250 online!\n");
  // }
  sensor->Probe();
  sensor->Initialize();
  
  //-------------------------------------------------------------------------
  // float ax, ay, az, gx, gy, gz, mx, my, mz, temperature;

  while (1)
  {
    // if (sensor->checkNewData())  {
    // sensor->readAccelerometer(ax, ay, az);
    // sensor->readGyrometer(gx, gy, gz);
    // sensor->readMagnetometer(mx, my, mz);
    // temperature = sensor->readTemperature();
    // }
    sensor->Update();
    std::cout << sensor->GetData() << std::endl;
    // auto data = sensor->GetData();
    // printf("Acc: %+7.3f %+7.3f %+7.3f\t", ax, ay, az);
    // printf("Gyr: %+7.3f %+7.3f %+7.3f\t", gx, gy, gz);
    // printf("Mag: %+7.3f %+7.3f %+7.3f\t", mx, my, mz);
    // printf("Temp: %+7.3f\n", temperature);

    usleep(500000);
  }
}