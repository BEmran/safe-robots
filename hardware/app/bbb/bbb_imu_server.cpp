
#include <getopt.h>
#include <signal.h>

#include <cstdlib>
#include <initializer_list>

#include "bbb/sensors/mpu/bbb_mpu9250.hpp"
#include "common/utils.hpp"
#include "core/utils/clock.hpp"
#include "core/utils/logger_macros.hpp"
#include "core/utils/math.hpp"
#include "core/utils/server_socket.hpp"

using ImuData = core::utils::ImuData;
using core::utils::ServerSocket;

constexpr float RAD_TO_DEG = 1. / core::utils::DEG_TO_RAD;
constexpr auto MaximumClientTries = 5;
int port_number = 1234;
using hardware::common::sensors::MS2_TO_G;
// constexpr double MS2_TO_G_ = 1.0 / GRAVITY;
// possible modes, user selected with command line arguments
enum class GyroMode { RAD, DEG, RAW };
enum class AccelMode { MS2, G, RAW };

// default values
bool gRUNNING{true};
bool gENABLE_MAG{true};
GyroMode gGMode = GyroMode::RAD;
AccelMode gAMode = AccelMode::MS2;

// printed if some invalid argument was given
void PrintHelpMessage() {
  printf("\n");
  printf("-r	print raw adc values instead of radians\n");
  printf("-d	print gyro in radians/s instead of degrees/s\n");
  printf("-g	print acceleration in G instead of m/s^2\n");
  printf("-m	print magnetometer data as well as accel/gyro\n");
  printf("-h	print this help message\n");
  printf("\n");
}

// interrupt handler to catch ctrl-c
void __signal_handler(__attribute__((unused)) int dummy) {
  gRUNNING = false;
  SYS_LOG_DEBUG("detected signal handler\n");
  return;
}

bool PrintHeaderMsg() {
  // print the header
  printf("\ntry 'rc_test_mpu -h' to see other options\n\n");
  switch (gAMode) {
    case AccelMode::MS2:
      printf("   Accel XYZ(m/s^2)  |");
      break;
    case AccelMode::G:
      printf("     Accel XYZ(G)    |");
      break;
    case AccelMode::RAW:
      printf("  Accel XYZ(raw ADC) |");
      break;
    default:
      SYS_LOG_WARN("ERROR: invalid accel mode\n");
      return false;
  }

  switch (gGMode) {
    case GyroMode::RAD:
      printf("   Gyro XYZ (rad/s)  |");
      break;
    case GyroMode::DEG:
      printf("   Gyro XYZ (deg/s)  |");
      break;
    case GyroMode::RAW:
      printf("  Gyro XYZ (raw ADC) |");
      break;
    default:
      SYS_LOG_WARN("ERROR: invalid gyro mode\n");
      return false;
  }
  if (gENABLE_MAG) {
    printf("  Mag Field XYZ(uT)  |");
  }
  printf(" Temp (C)\n");
  return true;
}

void PrintAccelValue(const ImuData data) {
  const double x = static_cast<double>(data.accel.data[0]);
  const double y = static_cast<double>(data.accel.data[1]);
  const double z = static_cast<double>(data.accel.data[2]);

  switch (gAMode) {
    case AccelMode::MS2:
      printf("%6.2f %6.2f %6.2f |", x, y, z);
      break;
    case AccelMode::G:
      printf("%6.2f %6.2f %6.2f |", x * MS2_TO_G, y * MS2_TO_G, z * MS2_TO_G);
      break;
    // case AccelMode::RAW:
    //   printf("%6d %6d %6d |", data.raw_accel[0], data.raw_accel[1],
    //          data.raw_accel[2]);
    default:
      SYS_LOG_ERROR("UNDEFINED gAMode");
  }
}

void PrintGyroValue(const ImuData data) {
  const double x = static_cast<double>(data.gyro.data[0]);
  const double y = static_cast<double>(data.gyro.data[1]);
  const double z = static_cast<double>(data.gyro.data[2]);
  const double conv = static_cast<double>(RAD_TO_DEG);
  switch (gGMode) {
    case GyroMode::RAD:
      printf("%6.1f %6.1f %6.1f |", x, y, z);
      break;
    case GyroMode::DEG:
      printf("%6.1f %6.1f %6.1f |", x * conv, y * conv, z * conv);
      break;
      // case GyroMode::RAW:
      // printf("%6d %6d %6d |", data.raw_gyro.data[0], data.raw_gyro.data[1],
      //        data.raw_gyro.data[2]);
    default:
      SYS_LOG_ERROR("UNDEFINED gGMode");
  }
}

void PrintMagValue(const ImuData data) {
  const double x = static_cast<double>(data.mag.data[0]);
  const double y = static_cast<double>(data.mag.data[1]);
  const double z = static_cast<double>(data.mag.data[2]);
  if (gENABLE_MAG) {
    printf("%6.1f %6.1f %6.1f |", x, y, z);
  }
}

void PrintTempValue(const ImuData data) {
  printf(" %4.1f    ", data.temp.value);
}

void SelectPort(const char* option) {
  port_number = std::atoi(option);
}

bool ParseOption(int argc, char* argv[]) {
  // parse arguments
  int c;
  opterr = 0;
  while ((c = getopt(argc, argv, "rdgmhp:")) != -1) {
    switch (c) {
      case 'r':
        gGMode = GyroMode::RAW;
        gAMode = AccelMode::RAW;
        printf("\nRaw values are from 16-bit ADC\n");
        break;
      case 'd':
        gGMode = GyroMode::RAD;
        break;
      case 'g':
        gAMode = AccelMode::G;
        break;
      case 'p':
        SelectPort(optarg);
        break;
      case 'm':
        gENABLE_MAG = false;
        break;
      case 'h':  // __fall_through__
      default:
        PrintHelpMessage();
        return false;
    }
  }
  return true;
}

int main(int argc, char* argv[]) {
  // set signal handler so the loop can exit cleanly
  signal(SIGINT, __signal_handler);

  if (not ParseOption(argc, argv)) {
    return EXIT_FAILURE;
  }

  hardware::common::sensors::mpu::Config config;
  hardware::bbb::sensors::mpu::BbbMpu9250 mpu(config);

  if (not mpu.Probe()) {
    SYS_LOG_ERROR("Failed to prop MPU sensor\n");
    return EXIT_FAILURE;
  }

  mpu.Initialize();

  if (not PrintHeaderMsg()) {
    return EXIT_FAILURE;
  }

  //-------------------------------------------------------------------------
  const auto begin = core::utils::TimeInMicroSeconds();
  auto tries = MaximumClientTries;
  constexpr auto send_delay = 100;

  std::unique_ptr<ServerSocket> server =
    std::make_unique<ServerSocket>(port_number);
  while (--tries > 0) {
    server->Accept();
    while (server->IsReady() && gRUNNING) {
      // printf("\r");
      // read sensor data
      mpu.Update();
      ImuData data = mpu.GetData();
      // // print data
      // PrintAccelValue(data);
      // PrintGyroValue(data);
      // PrintMagValue(data);
      // PrintTempValue(data);
      // // sleep 0.1 sec
      // fflush(stdout);
      std::stringstream ss;
      ss << core::utils::TimeInMicroSeconds() - begin << ", "  //
         << data.accel.data.x() << ", "                        //
         << data.accel.data.y() << ", "                        //
         << data.accel.data.z() << ", "                        //
         << data.gyro.data.x() << ", "                         //
         << data.gyro.data.y() << ", "                         //
         << data.gyro.data.z() << ", "                         //
         << data.mag.data.x() << ", "                          //
         << data.mag.data.y() << ", "                          //
         << data.mag.data.z() << ";";                          //
      std::cout << ss.str() << std::endl;
      (*server) << ss.str();
      hardware::common::MilliDelay(send_delay);
    }
    SYS_LOG_WARN("Lost connection");
  }
  printf("\n");
  return EXIT_SUCCESS;
}