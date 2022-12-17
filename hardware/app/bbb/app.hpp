// Copyright (C) 2022 Bara Emran - All Rights Reserved
#ifndef HARDWARE_APP_BBB_APP_HPP_
#define HARDWARE_APP_BBB_APP_HPP_

#include <getopt.h>
#include <signal.h>

#include <functional>
#include <memory>

#include "app_utils.hpp"
#include "bbb/sensors/mpu/bbb_mpu9250.hpp"
#include "core/utils/logger_macros.hpp"
#include "core/utils/server_socket.hpp"

using core::utils::ServerSocket;
using hardware::bbb::sensors::mpu::BbbMpu9250;
using hardware::common::sensors::mpu::Config;

constexpr int DEFAULT_PORT_NUMBER{1234};
bool RUNNING{false};

// printed if some invalid argument was given
void PrintHelpMessage() {
  SYS_LOG_INFO("\n")                                           //
    << "-r	\tsend raw values\n"                               //
    << "-d	\tsend gyro in radians/s instead of degrees/s\n"   //
    << "-g	\tsend acceleration in G instead of m/s^2\n"       //
    << "-p  \t[port number] is used to configure port number"  //
    << "-h	\tprint this help message\n"                       //
    << "\n";
}

// interrupt handler to catch ctrl-c
void SignalHandler(int /*dummy*/) {
  RUNNING = false;
  SYS_LOG_WARN("Detected signal handler ctrl+c");
  return;
}

struct App {
  std::unique_ptr<BbbMpu9250> sensor;
  std::unique_ptr<ServerSocket> server;
  Modes modes;
  int port_number{DEFAULT_PORT_NUMBER};

  App() : server{nullptr} {
    RUNNING = true;
    // set signal handler so the loop can exit cleanly
    signal(SIGINT, SignalHandler);
  }

  void InitializeServer() {
    server = std::make_unique<ServerSocket>(port_number);
  }

  bool ParseOption(int argc, char* argv[]) {
    // prevent the error message
    opterr = 0;
    // parse arguments
    int opt;
    while ((opt = getopt(argc, argv, "rdghp:")) != -1) {
      switch (opt) {
        case 'r':
          modes.gyro = GyroMode::RAW;
          modes.accel = AccelMode::RAW;
          modes.mag = MagMode::RAW;
          SYS_LOG_INFO("Using raw values");
          break;
        case 'd':
          modes.gyro = GyroMode::DEG;
          SYS_LOG_INFO("Using degree/sec as a unit for Gyroscope values");
          break;
        case 'g':
          modes.accel = AccelMode::G;
          SYS_LOG_INFO(
            "Using Gravitational Acceleration (G) as a unit for Accelerometer "
            "values");
          break;
        case 'p':
          port_number = std::atoi(optarg);
          break;
        case 'h':  // [[fall_through]]
          PrintHelpMessage();
          return false;
        default:
          PrintHelpMessage();
          SYS_LOG_ERROR("Wrong parameter is passed.");
          return false;
      }
    }

    SYS_LOG_INFO("\ntry '-h' to see other options\n");
    return true;
  }

  void InitializeSensor(const Config& config) {
    sensor = std::make_unique<BbbMpu9250>(config);
    if (not sensor->Reset()) {
      SYS_LOG_ERROR("Failed to reset MPU sensor\n");
    }
    if (not sensor->Probe()) {
      SYS_LOG_ERROR("Failed to prop MPU sensor\n");
    }

    sensor->Initialize();
  }
};

#endif  // NAVIO_APP_APP_HPP
