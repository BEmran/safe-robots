// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_DATA_HPP_
#define CORE_UTILS_DATA_HPP_

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "core/utils/math.hpp"

namespace core::utils {
constexpr int FULL_PRECISION = 10;
constexpr int HALF_PRECISION = 4;

struct Data {
 public:
  virtual ~Data() = default;
  virtual void Clear() = 0;
  virtual std::string Header() const = 0;
  virtual std::string ToString() const = 0;
};

std::string Header(std::vector<const Data*> vec);
std::string ToString(std::vector<const Data*> vec);

struct AdcData : public Data {
  Vec3 values = Vec3::Zero();

  inline void Clear() override {
    values.setZero();
  }
  inline std::string Header() const override {
    return "adc:0, adc:1, adc:2";
  }
  inline std::string ToString() const override {
    std::stringstream ss;
    ss << values.format(kVecFmtSimple);
    return ss.str();
  }
};

struct DoubleData : public Data {
  double value = 0.0;
  inline void Clear() override {
    value = 0.0;
  }
  inline std::string Header() const override {
    return "value";
  }
  inline std::string ToString() const override {
    return std::to_string(value);
  }
};

struct BarData : public DoubleData {
  inline std::string Header() const override {
    return "bar";
  }
};

struct TemperatureData : public DoubleData {
  inline std::string Header() const override {
    return "temp";
  }
};

struct HeadingData : public DoubleData {
  inline std::string Header() const override {
    return "heading";
  }
};

struct GpsData : public Data {
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;

  inline void Clear() override {
    lat = 0.0;
    lon = 0.0;
    alt = 0.0;
  }
  inline std::string Header() const override {
    return "gps:lat, gps:long, gps:alt";
  }
  inline std::string ToString() const override {
    std::stringstream ss;
    ss << std::to_string(lat) << ", " << std::to_string(lon) << ", "
       << std::to_string(alt);
    return ss.str();
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
  inline void Clear() override {
    data.setZero();
  }
  inline std::string Header() const override {
    return "data:x, data:y, data:z";
  }
  inline std::string ToString() const override {
    std::stringstream ss;
    ss << data.format(kVecFmtSimple);
    return ss.str();
  }
};

struct AccelData : public Vec3Data {
  inline std::string Header() const override {
    return "accel:x, accel:y, accel:z";
  }
};

struct GyroData : public Vec3Data {
  inline std::string Header() const override {
    return "gyro:x, gyro:y, gyro:z";
  }
};

struct MagData : public Vec3Data {
  inline std::string Header() const override {
    return "mag:x, mag:y, mag:z";
  }
};

struct RPYData : public Vec3Data {
  inline std::string Header() const override {
    return "roll, pitch, yaw";
  }
};

struct QuatData : public Data {
  Quat data = Quat::Identity();
  inline void Clear() override {
    data.setIdentity();
  }
  inline std::string Header() const override {
    return "quat:w, quat:x, quat:y, quat:z";
  }
  inline std::string ToString() const override {
    std::stringstream ss;
    ss << data.w() << ", " << data.vec().format(kVecFmtSimple);
    return ss.str();
  }
};

/**
 * @brief holds simple version of IMU sensor data
 *
 */
struct ImuData : public Data {
  ImuData() {
  }
  TemperatureData temp;  ///< thermometer, in units of degrees Celsius
  HeadingData heading;   ///< fused heading filtered with gyro and accel data,
                         ///< same as Tait-Bryan yaw in radians
  AccelData accel;       ///< accelerometer (XYZ) in units of m/s^2
  GyroData gyro;         ///< gyroscope (XYZ) in units of degrees/s
  MagData mag;           ///< magnetometer (XYZ) in units of uT
  QuatData quat;         ///< normalized quaternion
  RPYData tait_bryan;    ///< Tait-Bryan angles (roll pitch yaw) in radians
  std::vector<const Data*> vec{&accel,      &gyro,    &mag, &quat,
                               &tait_bryan, &heading, &temp};

  inline void Clear() override {
    temp.Clear();
    heading.Clear();
    accel.Clear();
    gyro.Clear();
    mag.Clear();
    tait_bryan.Clear();
    quat.Clear();
  }
  inline std::string Header() const override {
    return core::utils::Header(vec);
  }
  inline std::string ToString() const override {
    return core::utils::ToString(vec);
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
