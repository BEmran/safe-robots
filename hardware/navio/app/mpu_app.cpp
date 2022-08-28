// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <unistd.h>

#include <memory>
#include <string>

#include "core/utils/date_time.hpp"
#include "core/utils/writter_file.hpp"
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
int main(int /*argc*/, char** /*argv[]*/) {
  auto app = core::utils::CreateDefaultNode("app");
  app.LogDebug("running....");

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
    app.LogError("Can't launch the app");
    return EXIT_FAILURE;
  }
  sensor->Initialize();
  sensor->Calibrate();

  while (true) {
    sensor->Update();
    std::cout << sensor->GetData();
    navio::hardware_utils::Delay(500);
  }
  return EXIT_SUCCESS;
}
