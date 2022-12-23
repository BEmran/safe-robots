
#include "app.hpp"
#include "core/utils/logger_macros.hpp"
#include "core/utils/spinner.hpp"
#include "core/utils/writer_file.hpp"

int main(int argc, char* argv[]) {
  App app;
  app.Create();
  hardware::common::sensors::mpu::Config config;
  app.InitializeSensor(config);
  app.ParseOption(argc, argv);

  //-------------------------------------------------------------------------
  constexpr double hz = 50.;
  core::utils::Spinner spinner(hz);
  core::utils::FileWriter file("log.txt");
  // print header
  file << app.HeaderMsg();
  while (app.IsOk()) {
    // prepare data
    const std::string msg = app.ExtractData();
    // print data
    file << msg << std::endl;
    spinner.SpinOnce();
  }
  return EXIT_SUCCESS;
}