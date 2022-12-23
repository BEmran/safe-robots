// Copyright (C) 2022 Bara Emran - All Rights Reserved
#ifndef HARDWARE_APP_BBB_APP_HPP_
#define HARDWARE_APP_BBB_APP_HPP_

#include <functional>
#include <memory>

#include "app_utils.hpp"
#include "bbb/sensors/mpu/bbb_mpu9250.hpp"
#include "common/sensors/utils.hpp"
#include "core/utils/server_socket.hpp"

using core::utils::ServerSocket;
using hardware::bbb::sensors::mpu::BbbMpu9250;
using hardware::common::sensors::mpu::Config;

constexpr int DEFAULT_PORT_NUMBER{1234};

// printed if some invalid argument was given
void PrintHelpMessage();

// interrupt handler to catch ctrl-c
void SignalHandler(int dummy);

struct App {
  std::unique_ptr<BbbMpu9250> sensor;
  std::unique_ptr<ServerSocket> server;
  Modes modes;
  int port_number{DEFAULT_PORT_NUMBER};

  App();

  void InitializeServer();

  std::string ExtractData() const;

  std::string HeaderMsg() const;

  bool ParseOption(int argc, char* argv[]);

  void InitializeSensor(const Config& config);

  bool IsOk() const;

  void Create();
};

#endif  // NAVIO_APP_APP_HPP
