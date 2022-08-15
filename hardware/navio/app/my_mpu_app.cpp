#include "mpu/AK8963.hpp"
#include "mpu/mpu_gyro.hpp"
#include "navio/util.h"
#include <unistd.h>
#include <string>
#include <memory>

// static const MPUIMU::Ascale_t ASCALE = MPUIMU::AFS_16G;
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
  mpu::AK8963Config mag_config;
  mag_config.mode = MMODE;
  mag_config.scale = MSCALE;
  auto mag_sensor = std::make_unique<mpu::AK8963>(mag_config, true);

  mpu::GyroConfig gyro_config;
  gyro_config.sample_rate_divisor = SAMPLE_RATE_DIVISOR;
  gyro_config.scale = GSCALE;
  auto gyro_sensor = std::make_unique<mpu::MpuGyro>(gyro_config, true);
  
  // Start the MPU9250
  if (!gyro_sensor->Probe())
  {
    printf("Bad Gyro device ID\n");
    return EXIT_FAILURE;
  }
  if (!mag_sensor->Probe())
  {
    printf("Bad Magnetometer device ID\n");
    return EXIT_FAILURE;
  }

  gyro_sensor->Initialize();

  mag_sensor->Initialize();
  
  //-------------------------------------------------------------------------
  // float ax, ay, az, gx, gy, gz, mx, my, mz, temperature;

  while (true)
  {
    // if (sensor->checkNewData())  {
    // sensor->readAccelerometer(ax, ay, az);
    // sensor->readGyrometer(gx, gy, gz);
    // sensor->readMagnetometer(mx, my, mz);
    // temperature = sensor->readTemperature();
    // }
    mag_sensor->Update();
    gyro_sensor->Update();
    std::cout << gyro_sensor->GetData() << "\t" << mag_sensor->GetData() << std::endl;
    // auto data = sensor->GetData();
    // printf("Acc: %+7.3f %+7.3f %+7.3f\t", ax, ay, az);
    // printf("Gyr: %+7.3f %+7.3f %+7.3f\t", gx, gy, gz);
    // printf("Mag: %+7.3f %+7.3f %+7.3f\t", mx, my, mz);
    // printf("Temp: %+7.3f\n", temperature);

    usleep(500000);
  }
  return EXIT_SUCCESS;
}