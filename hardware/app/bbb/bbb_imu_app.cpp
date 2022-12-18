

#include "app.hpp"
#include "core/utils/logger_macros.hpp"
#include "core/utils/spinner.hpp"

int main(int argc, char* argv[]) {
  App app;
  app.ParseOption(argc, argv);
  HeaderMsg(app.modes);

  hardware::common::sensors::mpu::Config config;
  app.InitializeSensor(config);

  HeaderMsg(app.modes);
  core::utils::Spinner spinner(10);
  // now just wait, print_data will run
  while (app.IsOk()) {
    printf("\r");
    // read sensor data
    app.sensor->Update();
    ImuData data = app.sensor->GetData();
    SensorRawData raw = app.sensor->GetRawData();
    // print data
    std::string str = PrintValues(app.modes, data, raw);
    printf("%s", str.c_str());
    fflush(stdout);
    // sleep 0.1 sec
    spinner.SpinOnce();
  }

  printf("\n");
  return EXIT_SUCCESS;
}
