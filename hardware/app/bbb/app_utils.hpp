// Copyright (C) 2022 Bara Emran - All Rights Reserved
#ifndef HARDWARE_APP_BBB_APP_UTILS_HPP_
#define HARDWARE_APP_BBB_APP_UTILS_HPP_

#include <iostream>
#include <string>

#include "common/sensors/mpu/def.hpp"
#include "common/utils.hpp"
#include "core/utils/clock.hpp"
#include "core/utils/data.hpp"
#include "core/utils/logger_macros.hpp"
#include "core/utils/math.hpp"

constexpr float RAD_TO_DEG = 1.f / core::utils::DEG_TO_RAD;

using core::utils::ImuData;
using core::utils::Vec3;
using hardware::common::sensors::MS2_TO_G;
using hardware::common::sensors::mpu::SensorRawData;

enum class GyroMode { RAD, DEG, RAW };
enum class AccelMode { MS2, G, RAW };
enum class MagMode { RAW, UTesla };
struct Modes {
  GyroMode gyro{GyroMode::RAD};
  AccelMode accel{AccelMode::MS2};
  MagMode mag{MagMode::UTesla};
};

void AccelHeaderMsg(const AccelMode mode) {
  switch (mode) {
    case AccelMode::MS2:
      printf("  Accel XYZ(m/s^2)  |");
      break;
    case AccelMode::G:
      printf("    Accel XYZ(G)    |");
      break;
    case AccelMode::RAW:
      printf(" Accel XYZ(raw ADC) |");
      break;
    default:
      SYS_LOG_ERROR("invalid accel mode\n");
  }
}

void GyroHeaderMsg(const GyroMode mode) {
  switch (mode) {
    case GyroMode::RAD:
      printf("  Gyro XYZ(rad/s)  |");
      break;
    case GyroMode::DEG:
      printf("  Gyro XYZ(deg/s)  |");
      break;
    case GyroMode::RAW:
      printf(" Gyro XYZ(raw ADC) |");
      break;
    default:
      SYS_LOG_ERROR("invalid gyro mode\n");
  }
}

void MagHeaderMsg(const MagMode mode) {
  switch (mode) {
    case MagMode::RAW:
      printf(" Mag Field XYZ(raw ADC)  |");
      break;
    case MagMode::UTesla:
      printf("   Mag Field XYZ (uT)    |");
      break;
    default:
      SYS_LOG_ERROR("invalid mag mode\n");
  }
}

void HeaderMsg(const Modes modes) {
  std::string header;
  AccelHeaderMsg(modes.accel);
  GyroHeaderMsg(modes.gyro);
  MagHeaderMsg(modes.mag);
  printf(" Temp (C)\n");
}

void PrintAccelValue(const AccelMode mode, const Vec3& accel, const Vec3& raw) {
  const double x = static_cast<double>(accel[0]);
  const double y = static_cast<double>(accel[1]);
  const double z = static_cast<double>(accel[2]);
  const int xr = static_cast<int>(raw[0]);
  const int yr = static_cast<int>(raw[1]);
  const int zr = static_cast<int>(raw[2]);

  switch (mode) {
    case AccelMode::MS2:
      printf("%6.2f %6.2f %6.2f |", x, y, z);
      break;
    case AccelMode::G:
      printf("%6.2f %6.2f %6.2f |", x * MS2_TO_G, y * MS2_TO_G, z * MS2_TO_G);
      break;
    case AccelMode::RAW:
      printf("%6d %6d %6d |", xr, yr, zr);
      break;
    default:
      SYS_LOG_ERROR("invalid accel mode\n");
  }
}

void PrintGyroValue(const GyroMode mode, const Vec3& gyro, const Vec3& raw) {
  const double x = static_cast<double>(gyro[0]);
  const double y = static_cast<double>(gyro[1]);
  const double z = static_cast<double>(gyro[2]);
  const int xr = static_cast<int>(raw[0]);
  const int yr = static_cast<int>(raw[1]);
  const int zr = static_cast<int>(raw[2]);
  const double conv = static_cast<double>(RAD_TO_DEG);

  constexpr std::size_t size = 100;
  std::string buffer;
  buffer.resize(size);
  switch (mode) {
    case GyroMode::RAD:
      printf("%6.1f %6.1f %6.1f |", x, y, z);
      break;
    case GyroMode::DEG:
      printf("%6.1f %6.1f %6.1f |", x * conv, y * conv, z * conv);
      break;
    case GyroMode::RAW:
      printf("%6d %6d %6d |", xr, yr, zr);
      break;
    default:
      SYS_LOG_ERROR("invalid accel mode\n");
  }
}

void PrintMagValue(const MagMode mode, const Vec3& mag, const Vec3& raw) {
  const double x = static_cast<double>(mag[0]);
  const double y = static_cast<double>(mag[1]);
  const double z = static_cast<double>(mag[2]);
  const int xr = static_cast<int>(raw[0]);
  const int yr = static_cast<int>(raw[1]);
  const int zr = static_cast<int>(raw[2]);

  constexpr std::size_t size = 100;
  std::string buffer;
  buffer.resize(size);
  switch (mode) {
    case MagMode::UTesla:
      printf("%6.1f %6.1f %6.1f |", x, y, z);
      break;
    case MagMode::RAW:
      printf("%6d %6d %6d |", xr, yr, zr);
      break;
    default:
      SYS_LOG_ERROR("invalid mag mode\n");
  }
}

void PrintTempValue(const double temp) {
  printf("  %4.1f", temp);
}

void PrintValues(const Modes modes, const ImuData imu,
                 const SensorRawData raw) {
  PrintAccelValue(modes.accel, imu.accel.data, raw.accel);
  PrintGyroValue(modes.gyro, imu.gyro.data, raw.gyro);
  PrintMagValue(modes.mag, imu.mag.data, raw.mag);
  PrintTempValue(imu.temp.value);
  fflush(stdout);
}

std::string ExtractData(const Modes /*modes*/, const ImuData imu,
                        const SensorRawData /*raw*/) {
  const static uint64_t begin = core::utils::TimeInMicroSeconds();
  const uint64_t dt = core::utils::TimeInMicroSeconds() - begin;
  std::stringstream ss;
  ss << dt << ", "                  //
     << imu.accel.data.x() << ", "  //
     << imu.accel.data.y() << ", "  //
     << imu.accel.data.z() << ", "  //
     << imu.gyro.data.x() << ", "   //
     << imu.gyro.data.y() << ", "   //
     << imu.gyro.data.z() << ", "   //
     << imu.mag.data.x() << ", "    //
     << imu.mag.data.y() << ", "    //
     << imu.mag.data.z() << ";";    //
  return ss.str();
}

#endif  // NAVIO_APP_APP_HPP
