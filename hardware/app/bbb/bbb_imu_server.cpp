
#include "app.hpp"
#include "core/utils/logger_macros.hpp"
#include "core/utils/spinner.hpp"

constexpr auto MaximumClientTries = 5;

int main(int argc, char* argv[]) {
  App app;
  app.Create();
  hardware::common::sensors::mpu::Config config;
  app.InitializeSensor(config);
  app.ParseOption(argc, argv);

  app.InitializeServer();

  //-------------------------------------------------------------------------
  auto tries = MaximumClientTries;
  constexpr double hz = 10.;
  core::utils::Spinner spinner(hz);
  // HeaderMsg(app.modes);
  while (--tries > 0 && app.IsOk()) {
    app.server->Accept();
    while (app.server->IsReady() && app.IsOk()) {
      // prepare data
      const std::string msg = app.ExtractData();
      // print data
      SYS_LOG_INFO(msg);
      // send data through server
      (*app.server) << msg;
      // delay
      spinner.SpinOnce();
    }
    SYS_LOG_WARN("Lost connection");
  }
  return EXIT_SUCCESS;
}