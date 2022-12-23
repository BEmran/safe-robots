// Copyright (C) 2022 Bara Emran - All Rights Reserved
#ifndef HARDWARE_APP_BBB_APP_UTILS_HPP_
#define HARDWARE_APP_BBB_APP_UTILS_HPP_

#include <string>

#include "common/sensors/mpu/def.hpp"
#include "common/utils.hpp"
#include "core/utils/data.hpp"
#include "core/utils/math.hpp"

using core::utils::ImuData;
using core::utils::Vec3;
using hardware::common::sensors::mpu::SensorRawData;

enum class GyroMode { RAD, DEG, RAW };
enum class AccelMode { MS2, G, RAW };
enum class MagMode { RAW, UTesla };
struct Modes {
  GyroMode gyro{GyroMode::RAD};
  AccelMode accel{AccelMode::MS2};
  MagMode mag{MagMode::UTesla};
};

void AccelHeaderMsg(const AccelMode mode);

void GyroHeaderMsg(const GyroMode mode);

void MagHeaderMsg(const MagMode mode);

void HeaderMsg(const Modes modes);

std::string PrintAccelValue(const AccelMode mode, const Vec3& accel,
                            const Vec3& raw);

std::string PrintGyroValue(const GyroMode mode, const Vec3& gyro,
                           const Vec3& raw);

std::string PrintMagValue(const MagMode mode, const Vec3& mag, const Vec3& raw);

std::string PrintTempValue(const double temp);

std::string PrintValues(const Modes modes, const ImuData imu,
                        const SensorRawData raw);

std::string PrepareData(const ImuData imu);

std::string PrepareData(const SensorRawData raw);

std::string HeaderMsgWithUnit(const std::string accel_unit,
                              const std::string gyro_unit,
                              const std::string mag_unit);
std::string AccelUnit(const AccelMode mode);
std::string GyroUnit(const GyroMode mode);
std::string MagUnit(const MagMode mode);
#endif  // NAVIO_APP_APP_HPP
