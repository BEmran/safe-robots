

#include <core/utils/logger_macros.hpp>
#include <core/utils/spinner.hpp>

#include "app.hpp"

int main(int argc, char* argv[]) {
  App app;
  app.ParseOption(argc, argv);
  HeaderMsg(app.modes);

  hardware::common::sensors::mpu::Config config;
  app.InitializeSensor(config);

  HeaderMsg(app.modes);
  core::utils::Spinner spinner(10);
  // now just wait, print_data will run
  while (RUNNING) {
    printf("\r");
    // read sensor data
    app.sensor->Update();
    ImuData data = app.sensor->GetData();
    // print data
    PrintValues(app.modes, data, SensorRawData());
    // sleep 0.1 sec
    spinner.SpinOnce();
  }

  printf("\n");
  return EXIT_SUCCESS;
}
