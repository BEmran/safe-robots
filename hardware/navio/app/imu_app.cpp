// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>

#include "app.hpp"

int main(int argc, char* argv[]) {
  App app;
  app.node->GetLogger().Debug("running....");

  if (navio::hardware_utils::CheckApm()) {
    app.node->GetLogger().Error("APM is busy. Can't launch the app");
    return EXIT_FAILURE;
  }

  app.ExtractArgument(argc, argv);

  if (!app.sensor) {
    return EXIT_FAILURE;
  }

  if (!app.sensor->Probe()) {
    app.node->GetLogger().Error("Sensor can't be probed");
    return EXIT_FAILURE;
  }

  app.sensor->Initialize();
  app.node->GetLogger().Debug("Sensor is initialized successfully");
  // sensor->Calibrate();

  constexpr auto display_delay = 500;
  while (true) {
    app.sensor->Update();
    std::cout << app.sensor->GetData();
    navio::hardware_utils::Delay(display_delay);
  }
  return EXIT_SUCCESS;
}
