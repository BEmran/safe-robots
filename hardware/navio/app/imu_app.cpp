// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <unistd.h>

#include <hardware/navio/app/app.hpp>
#include <iostream>
#include <memory>
#include <string>

int main(int argc, char* argv[]) {
  App app;
  app.node->GetLogger()->LogDebug("running....");

  if (navio::hardware_utils::CheckApm()) {
    app.node->GetLogger()->LogError("APM is busy. Can't launch the app");
    return EXIT_FAILURE;
  }

  app.ExtractArgument(argc, argv);

  if (!app.sensor) {
    return EXIT_FAILURE;
  }

  if (!app.sensor->Probe()) {
    app.node->GetLogger()->LogError("Sensor can't be probed");
    return EXIT_FAILURE;
  }

  app.sensor->Initialize();
  app.node->GetLogger()->LogDebug("Sensor is initialized successfully");
  // sensor->Calibrate();

  constexpr auto display_delay = 500;
  while (true) {
    app.sensor->Update();
    std::cout << app.sensor->GetData();
    navio::hardware_utils::Delay(display_delay);
  }
  return EXIT_SUCCESS;
}
