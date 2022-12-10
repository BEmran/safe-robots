
#include <getopt.h>
#include <signal.h>

#include <cstdio>

#include "i2c.hpp"
#include "mpu.hpp"

// bus for Robotics Cape and BeagleboneBlue is 2
// change this for your platform
#define I2C_BUS 2

// possible modes, user selected with command line arguments
enum class GyroMode { G_MODE_RAD, G_MODE_DEG, G_MODE_RAW };
enum class AccelMode { A_MODE_MS2, A_MODE_G, A_MODE_RAW };

// default values
bool gRUNNING{true};
bool gENABLE_MAG{true};
bool gENABLE_THER{true};
bool gENABLE_WARN{true};
GyroMode gGMode = GyroMode::G_MODE_DEG;
AccelMode gAMode = AccelMode::A_MODE_MS2;

// printed if some invalid argument was given
void PrintHelpMessage() {
  printf("\n");
  printf("-a	print raw adc values instead of radians\n");
  printf("-r	print gyro in radians/s instead of degrees/s\n");
  printf("-g	print acceleration in G instead of m/s^2\n");
  printf("-m	print magnetometer data as well as accel/gyro\n");
  printf("-t	print thermometer data as well as accel/gyro\n");
  printf("-w	print i2c warnings\n");
  printf("-h	print this help message\n");
  printf("\n");
}

// interrupt handler to catch ctrl-c
void __signal_handler(__attribute__((unused)) int dummy) {
  gRUNNING = false;
  return;
}

bool PrintHeaderMsg() {
  // print the header
  printf("\ntry 'rc_test_mpu -h' to see other options\n\n");
  switch (gAMode) {
    case AccelMode::A_MODE_MS2:
      printf("   Accel XYZ(m/s^2)  |");
      break;
    case AccelMode::A_MODE_G:
      printf("     Accel XYZ(G)    |");
      break;
    case AccelMode::A_MODE_RAW:
      printf("  Accel XYZ(raw ADC) |");
      break;
    default:
      printf("ERROR: invalid mode\n");
      return false;
  }

  switch (gGMode) {
    case GyroMode::G_MODE_RAD:
      printf("   Gyro XYZ (rad/s)  |");
      break;
    case GyroMode::G_MODE_DEG:
      printf("   Gyro XYZ (deg/s)  |");
      break;
    case GyroMode::G_MODE_RAW:
      printf("  Gyro XYZ (raw ADC) |");
      break;
    default:
      printf("ERROR: invalid mode\n");
      return false;
  }

  if (gENABLE_MAG) {
    printf("  Mag Field XYZ(uT)  |");
  }

  if (gENABLE_THER) {
    printf(" Temp (C)");
  }

  printf("\n");
  return true;
}

void PrintAccelValue(const MpuData data) {
  switch (gAMode) {
    case AccelMode::A_MODE_MS2:
      printf("%6.2f %6.2f %6.2f |", data.accel[0], data.accel[1],
             data.accel[2]);
      break;
    case AccelMode::A_MODE_G:
      printf("%6.2f %6.2f %6.2f |", data.accel[0] * MS2_TO_G,
             data.accel[1] * MS2_TO_G, data.accel[2] * MS2_TO_G);
      break;
    case AccelMode::A_MODE_RAW:
      printf("%6d %6d %6d |", data.raw_accel[0], data.raw_accel[1],
             data.raw_accel[2]);
  }
}

void PrintGyroValue(const MpuData data) {
  switch (gGMode) {
    case GyroMode::G_MODE_RAD:
      printf("%6.1f %6.1f %6.1f |", data.gyro[0] * DEG_TO_RAD,
             data.gyro[1] * DEG_TO_RAD, data.gyro[2] * DEG_TO_RAD);
      break;
    case GyroMode::G_MODE_DEG:
      printf("%6.1f %6.1f %6.1f |", data.gyro[0], data.gyro[1], data.gyro[2]);
      break;
    case GyroMode::G_MODE_RAW:
      printf("%6d %6d %6d |", data.raw_gyro[0], data.raw_gyro[1],
             data.raw_gyro[2]);
  }
}

void PrintMagValue(const MpuData data) {
  if (gENABLE_MAG) {
    printf("%6.1f %6.1f %6.1f |", data.mag[0], data.mag[1], data.mag[2]);
  }
}

void PrintTempValue(const MpuData data) {
  if (gENABLE_THER) {
    printf(" %4.1f    ", data.temp);
  }
}

bool ParseOption(int argc, char* argv[]) {
  // parse arguments
  int c;
  opterr = 0;
  while ((c = getopt(argc, argv, "argmtwh")) != -1) {
    switch (c) {
      case 'a':
        gGMode = GyroMode::G_MODE_RAW;
        gAMode = AccelMode::A_MODE_RAW;
        printf("\nRaw values are from 16-bit ADC\n");
        break;
      case 'r':
        gGMode = GyroMode::G_MODE_RAD;
        break;
      case 'g':
        gAMode = AccelMode::A_MODE_G;
        break;
      case 'm':
        gENABLE_MAG = false;
        break;
      case 't':
        gENABLE_THER = false;
        break;
      case 'w':
        gENABLE_WARN = false;
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
    return -1;
  }

  // use defaults for now, except also enable magnetometer.
  MpuConfig conf = MpuDefaultConfig();
  conf.i2c_bus = I2C_BUS;
  conf.enable_magnetometer = gENABLE_MAG;
  conf.show_warnings = gENABLE_WARN;

  MPU mpu;
  if (not mpu.Initialize(conf)) {
    fprintf(stderr, "rc_mpu_initialize_failed\n");
    return -1;
  }

  if (not PrintHeaderMsg()) {
    return -1;
  }

  // now just wait, print_data will run
  while (gRUNNING) {
    printf("\r");
    // read sensor data
    MpuData data = mpu.ReadData();
    // print data
    PrintAccelValue(data);
    PrintGyroValue(data);
    PrintMagValue(data);
    PrintTempValue(data);
    // sleep 0.1 sec
    fflush(stdout);
    MicroSleep(100000);
  }

  printf("\n");
  mpu.PowerOff();
  return 0;
}