
#include "app.hpp"
#include "core/utils/spinner.hpp"

constexpr auto MaximumClientTries = 5;

int main(int argc, char* argv[]) {
  App app;
  app.ParseOption(argc, argv);

  hardware::common::sensors::mpu::Config config;
  app.InitializeSensor(config);
  app.InitializeServer();

  //-------------------------------------------------------------------------
  auto tries = MaximumClientTries;
  constexpr double hz = 10.;
  core::utils::Spinner spinner(hz);
  // HeaderMsg(app.modes);
  while (--tries > 0 && RUNNING) {
    app.server->Accept();
    while (app.server->IsReady() && RUNNING) {
      // read sensor data
      app.sensor->Update();
      const ImuData data = app.sensor->GetData();
      // prepare data
      const std::string msg = ExtractData(app.modes, data, SensorRawData());
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