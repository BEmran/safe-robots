// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "app.hpp"
#include "core/utils/clock.hpp"

constexpr auto MaximumClientTries = 5;

int main(int argc, char* argv[]) {
  App app;
  app.node->GetNodeLogger()->Debug("running....");

  if (navio::hardware_utils::CheckApm()) {
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

  //-------------------------------------------------------------------------
  core::utils::HighResolutionClock clock;
  const auto begin = clock.Now().InMicroSeconds();
  auto tries = MaximumClientTries;
  constexpr auto send_delay = 10;
  while (--tries > 0) {
    app.server->Accept();
    while (app.server->IsReady()) {
      app.sensor->Update();
      const auto data = app.sensor->GetData();
      std::stringstream ss;
      ss << clock.Now().InMicroSeconds() - begin << ", "  //
         << data.accel.data << ", "                       //
         << data.gyro.data << ", "                        //
         << data.mag.data << ", " << data.tait_bryan.data << ";";

      (*app.server) << ss.str();
      navio::hardware_utils::Delay(send_delay);
    }
    app.node->GetNodeLogger()->Warn("Lost connection");
  }

  return EXIT_SUCCESS;
}
