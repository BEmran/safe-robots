#include "mpu/mpu9250.hpp"
#include "navio/util.h"
#include <unistd.h>
#include <string>
#include <memory>
#include <core/utils/writter_file.hpp>
#include <core/utils/date_time.hpp>

constexpr mpu::AccelScale ASCALE = mpu::AccelScale::FS_16G;
constexpr mpu::AccelBandWidthHz ABW = mpu::AccelBandWidthHz::BW_44HZ;
constexpr mpu::GyroScale GSCALE = mpu::GyroScale::FS_2000DPS;
constexpr mpu::GyroBandWidthHz GBW = mpu::GyroBandWidthHz::BW_184HZ;
constexpr mpu::MagMode MMODE = mpu::MagMode::CONTINUES_100HZ;
constexpr mpu::MagScale MSCALE = mpu::MagScale::FS_16BITS;
constexpr uint8_t SAMPLE_RATE_DIVISOR = 4;

uint TimeInMilliSec()
{
  const auto now = std::chrono::system_clock::now();
  const auto epoch = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(epoch).count();
}
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
  auto spi = std::make_unique<SPI>(navio::MPU_SPI_PATH, false);
  auto sensor = std::make_unique<mpu::Mpu9250>(config, std::move(spi), true);

  if (!sensor->Probe())
  {
    return EXIT_FAILURE;
  }
  sensor->Initialize();

  core::utils::FileWritter file("imu.txt");
  // sensor->Calibrate();

  //-------------------------------------------------------------------------
  const auto begin = TimeInMilliSec();
  while (true)
  {
    // sensor->Update();
    // std::cout << sensor->GetData();
    // usleep(500000);

    auto raw = sensor->ReadRawData();
    std::stringstream ss;
    ss << TimeInMilliSec() - begin << ", "  //
       << raw.accel.x() << ", " << raw.accel.y() << ", " << raw.accel.z() << ", "  //
       << raw.gyro.x() << ", " << raw.gyro.y() << ", " << raw.gyro.z() << ", "  //
       << raw.mag.x() << ", " << raw.mag.y() << ", " << raw.mag.z() << ", "  //
       << raw.mag_over_flow;
    file.dump(ss.str());
    usleep(10000);
  }
  return EXIT_SUCCESS;
}