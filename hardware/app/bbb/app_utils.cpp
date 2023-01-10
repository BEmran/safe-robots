// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "app_utils.hpp"

#include <iostream>

#include "core/utils/clock.hpp"
#include "core/utils/logger_macros.hpp"

constexpr float RAD_TO_DEG = 1.f / core::math::DEG_TO_RAD;
using hardware::common::sensors::MS2_TO_G;

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

std::string PrintAccelValue(const AccelMode mode, const Vec3& accel,
                            const Vec3& raw) {
  const double x = static_cast<double>(accel.x());
  const double y = static_cast<double>(accel.y());
  const double z = static_cast<double>(accel.z());
  const int xr = static_cast<int>(raw.x());
  const int yr = static_cast<int>(raw.y());
  const int zr = static_cast<int>(raw.z());

  constexpr std::size_t size = 100;
  // std::string buffer;
  char buffer[size];
  // buffer.resize(size);
  switch (mode) {
    case AccelMode::MS2:
      snprintf(buffer, size, "%6.2f %6.2f %6.2f |", x, y, z);
      break;
    case AccelMode::G:
      snprintf(buffer, size, "%6.2f %6.2f %6.2f |", x * MS2_TO_G, y * MS2_TO_G,
               z * MS2_TO_G);
      break;
    case AccelMode::RAW:
      snprintf(buffer, size, "%6d %6d %6d |", xr, yr, zr);
      break;
    default:
      SYS_LOG_ERROR("invalid accel mode\n");
  }
  // printf("%s", buffer.c_str());
  return buffer;
}

std::string PrintGyroValue(const GyroMode mode, const Vec3& gyro,
                           const Vec3& raw) {
  const double x = static_cast<double>(gyro.x());
  const double y = static_cast<double>(gyro.y());
  const double z = static_cast<double>(gyro.z());
  const int xr = static_cast<int>(raw.x());
  const int yr = static_cast<int>(raw.y());
  const int zr = static_cast<int>(raw.z());
  const double conv = static_cast<double>(RAD_TO_DEG);

  constexpr std::size_t size = 100;
  // std::string buffer;
  char buffer[size];
  // buffer.resize(size);
  switch (mode) {
    case GyroMode::RAD:
      snprintf(buffer, size, "%6.1f %6.1f %6.1f |", x, y, z);
      break;
    case GyroMode::DEG:
      snprintf(buffer, size, "%6.1f %6.1f %6.1f |", x * conv, y * conv,
               z * conv);
      break;
    case GyroMode::RAW:
      snprintf(buffer, size, "%6d %6d %6d |", xr, yr, zr);
      break;
    default:
      SYS_LOG_ERROR("invalid accel mode\n");
  }
  // printf("%s", buffer.c_str());
  return buffer;
}

std::string PrintMagValue(const MagMode mode, const Vec3& mag,
                          const Vec3& raw) {
  const double x = static_cast<double>(mag.x());
  const double y = static_cast<double>(mag.y());
  const double z = static_cast<double>(mag.z());
  const int xr = static_cast<int>(raw.x());
  const int yr = static_cast<int>(raw.y());
  const int zr = static_cast<int>(raw.z());

  constexpr std::size_t size = 100;
  // std::string buffer;
  char buffer[size];
  // buffer.resize(size);
  switch (mode) {
    case MagMode::UTesla:
      snprintf(buffer, size, "%6.1f %6.1f %6.1f |", x, y, z);
      break;
    case MagMode::RAW:
      snprintf(buffer, size, "%6d %6d %6d |", xr, yr, zr);
      break;
    default:
      SYS_LOG_ERROR("invalid mag mode\n");
  }
  // printf("%s", buffer.c_str());
  return buffer;
}

std::string PrintTempValue(const double temp) {
  constexpr std::size_t size = 100;
  char buffer[size];
  // buffer.resize(size);
  snprintf(buffer, size, "  %4.1f", temp);
  // printf("%s", buffer.c_str());
  return buffer;
}

std::string PrintValues(const Modes modes, const ImuData imu,
                        const SensorRawData raw) {
  std::string str = PrintAccelValue(modes.accel, imu.accel.data, raw.accel) +
                    PrintGyroValue(modes.gyro, imu.gyro.data, raw.gyro) +
                    PrintMagValue(modes.mag, imu.mag.data, raw.mag) +
                    PrintTempValue(imu.temp.value);
  return str;
}

std::string PrepareData(const ImuData imu) {
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
     << imu.mag.data.z() << ","     //
     << imu.overflow << ";";        //
  return ss.str();
}

std::string PrepareData(const SensorRawData raw) {
  const static uint64_t begin = core::utils::TimeInMicroSeconds();
  const uint64_t dt = core::utils::TimeInMicroSeconds() - begin;
  std::stringstream ss;
  ss << dt << ", "                 //
     << raw.accel.x() << ", "      //
     << raw.accel.y() << ", "      //
     << raw.accel.z() << ", "      //
     << raw.gyro.x() << ", "       //
     << raw.gyro.y() << ", "       //
     << raw.gyro.z() << ", "       //
     << raw.mag.x() << ", "        //
     << raw.mag.y() << ", "        //
     << raw.mag.z() << ","         //
     << raw.mag_over_flow << ";";  //
  return ss.str();
}

std::string HeaderMsgWithUnit(const std::string accel_unit,
                              const std::string gyro_unit,
                              const std::string mag_unit) {
  ImuData imu;
  std::stringstream ss;
  ss << "dt (ms), "                                        //
     << imu.accel.Header() << " (" << accel_unit << "), "  //
     << imu.gyro.Header() << " (" << gyro_unit << "), "    //
     << imu.mag.Header() << " (" << mag_unit << "),"
     << "over-flow (bit);";  //
  return ss.str();
}

std::string AccelUnit(const AccelMode mode) {
  switch (mode) {
    case AccelMode::G:
      return "G";
      break;
    case AccelMode::MS2:
      return "m/s2";
      break;
    case AccelMode::RAW:
      return "bits";
      break;
    default:
      SYS_LOG_WARN("invalid accel mode\n");
      return "";
  }
}

std::string GyroUnit(const GyroMode mode) {
  switch (mode) {
    case GyroMode::DEG:
      return "deg/s";
      break;
    case GyroMode::RAD:
      return "rad/s";
      break;
    case GyroMode::RAW:
      return "bits";
      break;
    default:
      SYS_LOG_WARN("invalid gyro mode\n");
      return "";
  }
}

std::string MagUnit(const MagMode mode) {
  switch (mode) {
    case MagMode::UTesla:
      return "uTesla";
      break;
    case MagMode::RAW:
      return "bits";
      break;
    default:
      SYS_LOG_WARN("invalid mag mode\n");
      return "";
  }
}