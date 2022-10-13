// Copyright (C) 2022 Bara Emran - All Rights Reserved
#ifndef NAVIO_APP_APP_HPP
#define NAVIO_APP_APP_HPP

#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "core/utils/server_socket.hpp"
#include "navio/hardware_utils.hpp"
#include "sensors/lsm/lsm9ds1.hpp"
#include "sensors/mpu/mpu9250.hpp"

using core::sensors::SensorModuleAbs;
using core::utils::ImuData;
using core::utils::Node;
using core::utils::ServerSocket;
using navio::SPI;
using sensors::lsm::Lsm9ds1;
using sensors::mpu::Mpu9250;

struct App {
  std::unique_ptr<Node> node;
  std::unique_ptr<SensorModuleAbs<ImuData>> sensor;
  std::unique_ptr<ServerSocket> server;

  App()
    : node{std::make_unique<Node>(core::utils::CreateSystemNode("app"))}
    , sensor{nullptr} {
  }

  void SelectSensor(const std::string& option) {
    auto imu_node =
      std::make_unique<Node>(core::utils::CreateSystemNode("im"
                                                           "u"));
    if (option == "mpu") {
      node->GetLogger().Debug("Selected: MPU9250");
      sensors::mpu::Config config;
      auto spi =
        std::make_unique<SPI>(navio::hardware_utils::MPU_SPI_PATH, false);
      sensor =
        std::make_unique<Mpu9250>(config, std::move(spi), std::move(imu_node));
    } else if (option == "lsm") {
      node->GetLogger().Debug("Selected: LSM9DS1");
      sensors::lsm::Config config;
      auto spi_a_g =
        std::make_unique<SPI>(navio::hardware_utils::LSM_A_G_PATH, false);
      auto spi_mag =
        std::make_unique<SPI>(navio::hardware_utils::LSM_MAG_PATH, false);
      sensor = std::make_unique<Lsm9ds1>(config, std::move(spi_a_g),
                                         std::move(spi_mag), std::move(node));

    } else {
      node->GetLogger().Error("Unknown Sensor");
    }
  }

  void SelectPort(const char* option) {
    const auto port_number = std::atoi(option);
    server = std::make_unique<ServerSocket>(port_number);
  }

  void PrintHelp() {
    std::stringstream ss;
    ss << "Possible parameters:\n"
          "\t-i [sensor name] \t\t\t Select sensor. Sensors names: mpu is "
          "MPU9250, lsm is LSM9DS1\n"
          "\t-p [port number] \t\t\t Select port number\n"
          "\t-h               \t\t\t Print help message"
       << std::endl;
    node->GetLogger().Debug(ss.str());
  }

  void ExtractArgument(int argc, char* argv[]) {
    if (argc < 2) {
      node->GetLogger().Warn(
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

        case 'p':
          SelectPort(optarg);
          break;

        case 'h':
          PrintHelp();
          break;

        default:
          node->GetLogger().Error("Wrong parameter.");
      }
    }
  }
};

#endif  // NAVIO_APP_APP_HPP
