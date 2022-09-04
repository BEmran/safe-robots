// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>

#include "navio/hardware_utils.hpp"
#include "sensors/lsm/lsm9ds1.hpp"
#include "sensors/mpu/mpu9250.hpp"

auto app = core::utils::CreateSystemNode("app");
std::unique_ptr<core::sensors::SensorModuleAbs<core::utils::ImuData>> sensor =
  nullptr;

void SelectSensor(const std::string& option) {
  auto node =
    std::make_unique<core::utils::Node>(core::utils::CreateSystemNode("im"
                                                                      "u"));
  if (option == "mpu") {
    app.GetLogger()->LogDebug("Selected: MPU9250");
    sensors::mpu::Config config;
    auto spi =
      std::make_unique<navio::SPI>(navio::hardware_utils::MPU_SPI_PATH, false);
    sensor = std::make_unique<sensors::mpu::Mpu9250>(config, std::move(spi),
                                                     std::move(node));
  } else if (option == "lsm") {
    app.GetLogger()->LogDebug("Selected: LSM9DS1");
    sensors::lsm::Config config;
    auto spi_a_g =
      std::make_unique<navio::SPI>(navio::hardware_utils::LSM_A_G_PATH, false);
    auto spi_mag =
      std::make_unique<navio::SPI>(navio::hardware_utils::LSM_MAG_PATH, false);
    sensor = std::make_unique<sensors::lsm::Lsm94s1>(
      config, std::move(spi_a_g), std::move(spi_mag), std::move(node));

  } else {
    app.GetLogger()->LogError("Unknown Sensor");
  }
}

void PrintHelp() {
  std::stringstream ss;
  ss << "Possible parameters:\n"
        "\t-i [sensor name] \t\t\t Select sensor. Sensors names: mpu is "
        "MPU9250, lsm is LSM9DS1\n"
        "\t-h               \t\t\t Print help message"
     << std::endl;
  app.GetLogger()->LogDebug(ss.str());
}

void ExtractArgument(int argc, char* argv[]) {
  if (argc < 2) {
    app.GetLogger()->LogDebug(
      "Please provide input information (mpu/lsm) and port");
    PrintHelp();
  }
  // prevent the error message
  opterr = 0;
  int parameter;

  while ((parameter = getopt(argc, argv, "i:p:h")) != -1) {
    switch (parameter) {
      case 'i':
        SelectSensor(std::string(optarg));
        break;

      case 'h':
        PrintHelp();
        break;

      default:
        app.GetLogger()->LogError("Wrong parameter.");
    }
  }
}

//=============================================================================
int main(int argc, char* argv[]) {
  auto app = core::utils::CreateSystemNode("app");
  app.GetLogger()->LogDebug("running....");

  if (navio::hardware_utils::CheckApm()) {
    app.GetLogger()->LogError("APM is busy. Can't launch the app");
    return EXIT_FAILURE;
  }

  ExtractArgument(argc, argv);

  if (!sensor) {
    return EXIT_FAILURE;
  }

  if (!sensor->Probe()) {
    app.GetLogger()->LogError("Sensor can't be probed");
    return EXIT_FAILURE;
  }

  sensor->Initialize();
  app.GetLogger()->LogDebug("Sensor is initialized successfully");
  // sensor->Calibrate();

  constexpr auto display_delay = 500;
  while (true) {
    sensor->Update();
    std::cout << sensor->GetData();
    navio::hardware_utils::Delay(display_delay);
  }
  return EXIT_SUCCESS;
}
