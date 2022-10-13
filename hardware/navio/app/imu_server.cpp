// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "app.hpp"
#include "core/utils/date_time.hpp"

constexpr auto MaximumClientTries = 5;

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

  //-------------------------------------------------------------------------
  const auto begin = core::utils::TimeInMilliSec();
  auto tries = MaximumClientTries;
  constexpr auto send_delay = 10;
  while (--tries > 0) {
    app.server->Accept();
    while (app.server->IsReady()) {
      app.sensor->Update();
      const auto data = app.sensor->GetData();
      std::stringstream ss;
      ss << core::utils::TimeInMilliSec() - begin << ", "  //
         << data.accel.data << ", "                        //
         << data.gyro.data << ", "                         //
         << data.mag.data << ", " << data.tait_bryan.data << ";";

      (*app.server) << ss.str();
      navio::hardware_utils::Delay(send_delay);
    }
    app.node->GetLogger().Warn("Lost connection");
  }

  return EXIT_SUCCESS;
}
