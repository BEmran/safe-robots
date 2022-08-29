// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_DATA_HPP_
#define CORE_UTILS_DATA_HPP_

#include <iomanip>
#include <iostream>

#include "core/utils/math.hpp"

namespace core::utils {
constexpr int FULL_PRECISION = 10;
constexpr int HALF_PRECISION = 4;

struct Data {
 public:
  virtual ~Data() = default;
  virtual void Clear() = 0;
};

struct AdcData : public Data {
  Vec3 values = Vec3::Zero();

  void Clear() override {
    values.setZero();
  }
};

struct DoubleData : public Data {
  double value = 0.0;
  void Clear() override {
    value = 0.0;
  }
};

struct BarData : public DoubleData {};

struct TemperatureData : public DoubleData {};

struct HeadingData : public DoubleData {};

struct GpsData : public Data {
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;

  void Clear() override {
    lat = 0.0;
    lon = 0.0;
    alt = 0.0;
  }
};

struct Vec3Data : public Data {
  Vec3 data = Vec3::Zero();
  Vec3Data() : data{0, 0, 0} {
  }
  explicit Vec3Data(const Vec3& vec) : data(vec) {
  }
  Vec3Data(MATH_TYPE x, MATH_TYPE y, MATH_TYPE z) : data{x, y, z} {
  }
  void Clear() override {
    data.setZero();
  }
};

struct AccelData : public Vec3Data {};

struct GyroData : public Vec3Data {};

struct MagData : public Vec3Data {};

struct RPYData : public Vec3Data {};

struct QuatData : public Data {
  Quat data = Quat::Identity();
  void Clear() override {
    data.Identity();
  }
};

/**
 * @brief holds simple version of IMU sensor data
 *
 */
struct ImuData : public Data {
  TemperatureData temp;  ///< thermometer, in units of degrees Celsius
  HeadingData heading;   ///< fused heading filtered with gyro and accel data,
                         ///< same as Tait-Bryan yaw in radians
  AccelData accel;       ///< accelerometer (XYZ) in units of m/s^2
  GyroData gyro;         ///< gyroscope (XYZ) in units of degrees/s
  MagData mag;           ///< magnetometer (XYZ) in units of uT
  QuatData quat;         ///< normalized quaternion
  RPYData tait_bryan;    ///< Tait-Bryan angles (roll pitch yaw) in radians

  void Clear() override {
    temp.Clear();
    heading.Clear();
    accel.Clear();
    gyro.Clear();
    mag.Clear();
    tait_bryan.Clear();
    quat.Clear();
  }
};

}  // namespace core::utils

namespace cu = core::utils;

/* print data details */
std::ostream& operator<<(std::ostream& os, const cu::AdcData& adc);
std::ostream& operator<<(std::ostream& os, const cu::BarData& bar);
std::ostream& operator<<(std::ostream& os, const cu::TemperatureData& temp);
std::ostream& operator<<(std::ostream& os, const cu::HeadingData& head);
std::ostream& operator<<(std::ostream& os, const cu::GpsData& gps);
std::ostream& operator<<(std::ostream& os, const cu::Vec3Data& vec);
std::ostream& operator<<(std::ostream& os, const cu::QuatData& quat);
std::ostream& operator<<(std::ostream& os, const cu::AccelData& accel);
std::ostream& operator<<(std::ostream& os, const cu::GyroData& gyro);
std::ostream& operator<<(std::ostream& os, const cu::MagData& mag);
std::ostream& operator<<(std::ostream& os, const cu::RPYData& rpy);
std::ostream& operator<<(std::ostream& os, const cu::ImuData& imu);

#endif  // CORE_UTILS_DATA_HPP_
