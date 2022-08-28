#ifndef CORE_UTILS_DATA_HPP
#define CORE_UTILS_DATA_HPP

#include <iomanip>
#include <iostream>

#include "core/utils/math.hpp"

namespace core::utils {
constexpr int FULL_PRECISION = 10;
constexpr int HALF_PRECISION = 4;

struct Data {
 public:
  virtual ~Data(){};
  virtual void Clear() = 0;
};

struct AdcData : public Data {
  Vec3 values = Vec3::Zero();

  void Clear() override {
    values.setZero();
  }
};

std::ostream& operator<<(std::ostream& os, const AdcData& adc);

struct DoubleData : public Data {
  double value = 0.0;
  void Clear() override {
    value = 0.0;
  }
};

struct BarData : public DoubleData {};

std::ostream& operator<<(std::ostream& os, const BarData& bar);

struct TemperatureData : public DoubleData {};

std::ostream& operator<<(std::ostream& os, const TemperatureData& temp);

struct HeadingData : public DoubleData {};

std::ostream& operator<<(std::ostream& os, const HeadingData& head);

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

std::ostream& operator<<(std::ostream& os, const GpsData& gps);

struct Vec3Data : public Data {
  Vec3 data = Vec3::Zero();
  Vec3Data() : data{0, 0, 0} {
  }
  explicit Vec3Data(const Vec3& vec) : data(vec) {
  }
  Vec3Data(const MATH_TYPE x, const MATH_TYPE y, const MATH_TYPE z)
    : data{x, y, z} {
  }
  void Clear() override {
    data.setZero();
  }
};

/**
 * @brief print Vec3 data details
 *
 */
std::ostream& operator<<(std::ostream& os, const Vec3Data& vec);

struct AccelData : public Vec3Data {};

struct GyroData : public Vec3Data {};

struct MagData : public Vec3Data {};

struct RPYData : public Vec3Data {};

/**
 * @brief print accelerometer data details
 *
 */
std::ostream& operator<<(std::ostream& os, const AccelData& accel);

/**
 * @brief print gyroscope data details
 *
 */
std::ostream& operator<<(std::ostream& os, const GyroData& gyro);

/**
 * @brief print magnetometer data details
 *
 */
std::ostream& operator<<(std::ostream& os, const MagData& mag);

/**
 * @brief print roll-pitch-yaw data details
 *
 */
std::ostream& operator<<(std::ostream& os, const RPYData& rpy);

struct QuatData : public Data {
  Quat data = Quat::Identity();
  void Clear() override {
    data.Identity();
  }
};

/**
 * @brief print quat data details
 *
 */
std::ostream& operator<<(std::ostream& os, const QuatData& quat);

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

/**
 * @brief print imu data details
 *
 */
std::ostream& operator<<(std::ostream& os, const ImuData& imu);

}  // namespace core::utils
#endif  // CORE_UTILS_DATA_HPP
