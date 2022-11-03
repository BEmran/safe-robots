// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>

#include "app.hpp"

int main(int argc, char* argv[]) {
  App app;
  app.node->GetNodeLogger()->Debug("running....");

  if (bbb::hardware_utils::CheckApm()) {
    app.node->GetNodeLogger()->Error("APM is busy. Can't launch the app");
    return EXIT_FAILURE;
  }

  app.ExtractArgument(argc, argv);

  if (!app.sensor) {
    return EXIT_FAILURE;
  }

  if (!app.sensor->Probe()) {
    app.node->GetNodeLogger()->Error("Sensor can't be probed");
    return EXIT_FAILURE;
  }

  app.sensor->Initialize();
  app.node->GetNodeLogger()->Debug("Sensor is initialized successfully");
  // sensor->Calibrate();

  constexpr auto display_delay = 500;
  while (true) {
    app.sensor->Update();
    std::cout << app.sensor->GetData();
    bbb::hardware_utils::Delay(display_delay);
  }
  return EXIT_SUCCESS;
}
