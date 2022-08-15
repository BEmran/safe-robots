#include "mpu/AK8963.hpp"
#include "mpu/mpu_gyro.hpp"
#include "navio/util.h"
#include <unistd.h>
#include <string>
#include <memory>

// static const MPUIMU::Ascale_t ASCALE = MPUIMU::AFS_16G;
static const mpu::GyroScale GSCALE = mpu::GyroScale::GFS_2000DPS;
static const mpu::MagScale MSCALE = mpu::MagScale::MFS_16BITS;
static const mpu::MagMode MMODE = mpu::MagMode::CONTINUES_100HZ_MODE;
static const uint8_t SAMPLE_RATE_DIVISOR = 4;

//=============================================================================
int main(int /*argc*/, char** /*argv[]*/)
{
  if (CheckApm())
  {
    return 1;
  }
  mpu::AK8963Config mag_config;
  mag_config.mode = MMODE;
  mag_config.scale = MSCALE;
  auto Mag = std::make_unique<mpu::AK8963>(mag_config, true);

  mpu::GyroConfig gyro_config;
  gyro_config.sample_rate_divisor = SAMPLE_RATE_DIVISOR;
  gyro_config.scale = GSCALE;
  auto Gyro = std::make_unique<mpu::MpuGyro>(gyro_config, true);
  
  // Start the MPU9250
  if (!Gyro->Probe())
  {
    printf("Bad Gyro device ID\n");
    // return EXIT_FAILURE;
  }
  if (!Mag->Probe())
  {
    printf("Bad Magnetometer device ID\n");
    // return EXIT_FAILURE;
  }

  Gyro->Initialize();

  Mag->Initialize();
  
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
    Mag->Update();
    Gyro->Update();
    std::cout << Gyro->GetData() << "\t" << Mag->GetData() << std::endl;
    // auto data = sensor->GetData();
    // printf("Acc: %+7.3f %+7.3f %+7.3f\t", ax, ay, az);
    // printf("Gyr: %+7.3f %+7.3f %+7.3f\t", gx, gy, gz);
    // printf("Mag: %+7.3f %+7.3f %+7.3f\t", mx, my, mz);
    // printf("Temp: %+7.3f\n", temperature);

    usleep(500000);
  }
}