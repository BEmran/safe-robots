#include <unistd.h>

#include <core/utils/date_time.hpp>
#include <core/utils/server_socket.hpp>
#include <core/utils/writter_file.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "navio/hardware_utils.hpp"
#include "sensors/mpu/mpu9250.hpp"

constexpr sensors::mpu::AccelScale ASCALE = sensors::mpu::AccelScale::FS_16G;
constexpr sensors::mpu::AccelBandWidthHz ABW =
  sensors::mpu::AccelBandWidthHz::BW_44HZ;
constexpr sensors::mpu::GyroScale GSCALE = sensors::mpu::GyroScale::FS_2000DPS;
constexpr sensors::mpu::GyroBandWidthHz GBW =
  sensors::mpu::GyroBandWidthHz::BW_184HZ;
constexpr sensors::mpu::MagMode MMODE = sensors::mpu::MagMode::CONTINUES_100HZ;
constexpr sensors::mpu::MagScale MSCALE = sensors::mpu::MagScale::FS_16BITS;
constexpr uint8_t SAMPLE_RATE_DIVISOR = 4;

//=============================================================================
int main(int argc, char* argv[]) {
  auto app = core::utils::CreateDefaultNode("app");
  app.LogDebug("running....");

  if (argc < 2) {
    app.LogError("no port provided");
    return EXIT_FAILURE;
  }

  const auto port = atoi(argv[1]);

  if (navio::hardware_utils::CheckApm()) {
    app.LogError("APM is busy. Can't launch the app");
    return EXIT_FAILURE;
  }

  auto node =
    std::make_unique<core::utils::Node>(core::utils::CreateDefaultNode("im"
                                                                       "u"));

  sensors::mpu::Config config;
  config.accel_bw = ABW;
  config.accel_scale = ASCALE;
  config.gyro_bw = GBW;
  config.gyro_scale = GSCALE;
  config.mag_mode = MMODE;
  config.mag_scale = MSCALE;
  config.sample_rate_divisor = SAMPLE_RATE_DIVISOR;
  auto spi =
    std::make_unique<navio::SPI>(navio::hardware_utils::MPU_SPI_PATH, false);
  auto sensor = std::make_unique<sensors::mpu::Mpu9250>(config, std::move(spi),
                                                        std::move(node));

  if (!sensor->Probe()) {
    app.LogError("MPU sensor can't be probed");
    return EXIT_FAILURE;
  }

  sensor->Initialize();
  app.LogDebug("MPU is initialized successfully");

  core::utils::FileWritter file("imu.txt");
  //-------------------------------------------------------------------------
  // Create the socket
  core::utils::ServerSocket server(port);

  const auto begin = core::utils::TimeInMilliSec();
  auto tries = 5;

  while (--tries > 0) {
    server.Accept();
    while (server.IsReady()) {
      const auto raw = sensor->ReadRawData();
      std::stringstream ss;
      ss << core::utils::TimeInMilliSec() - begin << ", "  //
         << raw.accel.x() << ", " << raw.accel.y() << ", " << raw.accel.z()
         << ", "  //
         << raw.gyro.x() << ", " << raw.gyro.y() << ", " << raw.gyro.z()
         << ", "  //
         << raw.mag.x() << ", " << raw.mag.y() << ", " << raw.mag.z()
         << ", "  //
         << raw.mag_over_flow << ";";

      server << ss.str();
      navio::hardware_utils::Delay(10);
    }
    app.LogWarn("Lost connection");
  }

  return EXIT_SUCCESS;
}
