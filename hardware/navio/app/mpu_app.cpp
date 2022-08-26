#include "mpu/MPU9250_Master_SPI.h"
#include "navio/util.h"
#include <unistd.h>
#include <string>
#include <memory>

static const MPUIMU::Ascale_t ASCALE = MPUIMU::AFS_16G;
static const MPUIMU::Gscale_t GSCALE = MPUIMU::GFS_2000DPS;
static const MPU9250::Mscale_t MSCALE = MPU9250::MFS_16BITS;
static const MPU9250::Mmode_t MMODE = MPU9250::M_100Hz;
static const uint8_t SAMPLE_RATE_DIVISOR = 4;

//=============================================================================
int main(int /*argc*/, char** /*argv[]*/)
{
  if (navio::CheckApm())
  {
    return 1;
  }

  auto sensor = std::make_unique<MPU9250_Master_SPI>(ASCALE, GSCALE, MSCALE, MMODE, SAMPLE_RATE_DIVISOR);

  // Start the MPU9250
  switch (sensor->begin())
  {
    case MPUIMU::ERROR_IMU_ID:
      printf("Bad IMU device ID\n");
    case MPUIMU::ERROR_MAG_ID:
      printf("Bad magnetometer device ID\n");
    case MPUIMU::ERROR_SELFTEST:
      printf("Failed self-test\n");
      break;
      return EXIT_FAILURE;
    default:
      printf("MPU9250 online!\n");
  }
// return EXIT_FAILURE;
  //-------------------------------------------------------------------------
  float ax, ay, az, gx, gy, gz, mx, my, mz, temperature;

  while (1)
  {
    // if (sensor->checkNewData())  {
    sensor->readAccelerometer(ax, ay, az);
    sensor->readGyrometer(gx, gy, gz);
    sensor->readMagnetometer(mx, my, mz);
    temperature = sensor->readTemperature();
    // }

    // auto data = sensor->GetData();
    printf("Acc: %+7.3f %+7.3f %+7.3f\t", ax, ay, az);
    printf("Gyr: %+7.3f %+7.3f %+7.3f\t", gx, gy, gz);
    printf("Mag: %+7.3f %+7.3f %+7.3f\t", mx, my, mz);
    printf("Temp: %+7.3f\n", temperature);

    usleep(500000);
  }
}