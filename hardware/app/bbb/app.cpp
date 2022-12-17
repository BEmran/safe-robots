
#include "app.hpp"

#include <getopt.h>
#include <signal.h>

#include "app_utils.hpp"
#include "core/utils/logger_macros.hpp"
#include "dependencies/yaml.hpp"

static bool RUNNING{false};

// printed if some invalid argument was given
void PrintHelpMessage() {
  SYS_LOG_INFO("\n")                                             //
    << "-r	\tsend raw values\n"                                 //
    << "-d	\tsend gyro in radians/s instead of degrees/s\n"     //
    << "-g	\tsend acceleration in G instead of m/s^2\n"         //
    << "-a  \tTo calibrate the accelerometer sensor\n"           //
    << "-w  \tTo calibrate the gyroscopic sensor\n"              //
    << "-m  \tTo calibrate the magnetometer sensor\n"            //
    << "-p  \t[port number] is used to configure port number\n"  //
    << "-h	\tprint this help message\n"                         //
    << "\n";
}

// interrupt handler to catch ctrl-c
void SignalHandler(int /*dummy*/) {
  RUNNING = false;
  SYS_LOG_WARN("Detected signal handler ctrl+c");
  return;
}

App::App() : server{nullptr} {
  RUNNING = true;
  // set signal handler so the loop can exit cleanly
  signal(SIGINT, SignalHandler);
}

void App::InitializeServer() {
  server = std::make_unique<ServerSocket>(port_number);
}

std::string App::ExtractData() const {
  if (modes.accel == AccelMode::RAW && modes.gyro == GyroMode::RAW &&
      modes.mag == MagMode::RAW) {
    const SensorRawData raw = sensor->ReadRawData();
    return PrepareData(raw);
    // todo: check what unit the user selected
  } else {
    sensor->Update();
    const ImuData data = sensor->GetData();
    return PrepareData(data);
  }
}

bool App::ParseOption(int argc, char* argv[]) {
  // prevent the error message
  opterr = 0;
  // parse arguments
  int opt;
  while ((opt = getopt(argc, argv, "rdgawmhp:")) != -1) {
    switch (opt) {
      case 'r':
        modes.gyro = GyroMode::RAW;
        modes.accel = AccelMode::RAW;
        modes.mag = MagMode::RAW;
        SYS_LOG_INFO("Using raw values");
        break;
      case 'd':
        modes.gyro = GyroMode::DEG;
        SYS_LOG_INFO("Using degree/sec as a unit for Gyroscope values");
        break;
      case 'g':
        modes.accel = AccelMode::G;
        SYS_LOG_INFO(
          "Using Gravitational Acceleration (G) as a unit for Accelerometer "
          "values");
        break;
      case 'a':
        SYS_LOG_INFO("Calibrate accelerometer sensor...");
        if (sensor) {
          sensor->CalibrateAccelerometer();
        }
        break;
      case 'w':
        SYS_LOG_INFO("Calibrate gyroscope sensor...");
        if (sensor) {
          sensor->CalibrateGyroscope();
        }
        break;
      case 'm':
        SYS_LOG_INFO("Calibrate magnetometer sensor...");
        if (sensor) {
          sensor->CalibrateMagnetometer();
        }
        break;
      case 'p':
        port_number = std::atoi(optarg);
        break;
      case 'h':  // [[fall_through]]
        PrintHelpMessage();
        return false;
      default:
        PrintHelpMessage();
        SYS_LOG_ERROR("Wrong parameter is passed.");
        return false;
    }
  }

  SYS_LOG_INFO("try use '-h' option to see other options");
  return true;
}

void App::InitializeSensor(const Config& config) {
  sensor = std::make_unique<BbbMpu9250>(config);
  if (not sensor->Reset()) {
    SYS_LOG_ERROR("Failed to reset MPU sensor\n");
  }
  if (not sensor->Probe()) {
    SYS_LOG_ERROR("Failed to prop MPU sensor\n");
  }

  sensor->Initialize();
}

bool App::IsOk() const {
  return RUNNING;
}

std::vector<std::string> ToString(core::utils::InputMat mat) {
  std::vector<float> vec(mat.data(), mat.data() + mat.size());
  std::vector<std::string> str(vec.size());
  std::transform(vec.begin(), vec.end(), str.begin(),
                 [](const float v) { return std::to_string(v); });
  return str;
}

void App::Create() {
  hardware::common::sensors::SensorSpecs<3> spec1;
  spec1.bias = core::utils::Vec3(1.2f, 3.44f, 5.666f);
  spec1.offset = core::utils::Vec3(-7.8f, -8.99f, -10.11111f);
  spec1.misalignment = core::utils::Mat3::Random();
  yaml::Sequence bias("bias", yaml::EntreeType::FLOAT, ToString(spec1.bias));
  yaml::Sequence offset("offset", yaml::EntreeType::FLOAT,
                        ToString(spec1.offset));
  yaml::Sequence misalignment("misalignment", yaml::EntreeType::FLOAT,
                              ToString(spec1.misalignment));
  yaml::Structure list_accel("accel", {&bias, &offset, &misalignment});
  yaml::Structure list_gyro("gyro", {&bias, &offset, &misalignment});
  yaml::Structure list_mag("mag", {&bias, &offset, &misalignment});
  yaml::Structure file("", {&list_accel, &list_gyro, &list_mag});
  std::cout << file.Print() << std::endl;
}
