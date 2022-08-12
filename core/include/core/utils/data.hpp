#ifndef CORE_UTILS_DATA_HPP
#define CORE_UTILS_DATA_HPP

#include <iomanip>
#include <iostream>
#include <array>
#include <algorithm>
#include <assert.h>

#include "core/utils/math.hpp"

namespace core::utils
{
constexpr int COUT_PRECISION = 10;

struct Data
{
 public:
  virtual ~Data(){};
  virtual void Clear() = 0;
};

struct AdcData : public Data
{
  Vec3 values = Vec3::Zero();

  void Clear() override
  {
    values.setZero();
  }
};

std::ostream& operator<<(std::ostream& os, const AdcData& adc)
{
  return os << "ADC data: " << adc.values.transpose() << std::endl;
}

struct BarData : public Data
{
  double value = 0.0;
  void Clear() override
  {
    value = 0.0;
  }
};

std::ostream& operator<<(std::ostream& os, const BarData& bar)
{
  return os << "Barometer data: " << bar.value << std::endl;
}

struct GpsData : public Data
{
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;

  void Clear() override
  {
    lat = 0.0;
    lon = 0.0;
    alt = 0.0;
  }
};

std::ostream& operator<<(std::ostream& os, const GpsData& gps)
{
  return os << "GPS data: " << std::setprecision(COUT_PRECISION) /* precision */
            << "\n- Lat: " << gps.lat                        /* latitude */
            << "\n- Lon: " << gps.lon                        /* longitude */
            << "\n- Alt: " << gps.alt << std::endl;          /* altitude */
}

/**
 * @brief holds simple version of IMU sensor data
 *
 */
struct ImuData : public Data
{
  double temp = 0.0;     ///< thermometer, in units of degrees Celsius
  double heading = 0.0;  ///< fused heading filtered with gyro and accel data,
                         ///< same as Tait-Bryan yaw in radians
  Vec3 accel = Vec3::Zero();     ///< accelerometer (XYZ) in units of m/s^2
  Vec3 gyro = Vec3::Zero();      ///< gyroscope (XYZ) in units of degrees/s
  Vec3 mag = Vec3::Zero();       ///< magnetometer (XYZ) in units of uT
  Quat quat = Quat::Identity();  ///< normalized quaternion
  Vec3 tait_bryan =
      Vec3::Zero();  ///< Tait-Bryan angles (roll pitch yaw) in radians

  void Clear() override
  {
    temp = 0.0;
    heading = 0.0;
    accel.setZero();
    gyro.setZero();
    mag.setZero();
    tait_bryan.setZero();
    quat.setIdentity();
  }
};

/**
 * @brief print imu data details
 *
 */
std::ostream& operator<<(std::ostream& os, const ImuData& imu)
{
  return os << "IMU data:"
            << "\n- Accel XYZ(m/s^2): " << imu.accel.transpose()
            << "\n- Gyro  XYZ(rad/s): " << imu.gyro.transpose()
            << "\n- Mag Field XYZ(uT): " << imu.mag.transpose()
            << "\n- Quat WXYZ: " << imu.quat
            << "\n- TaitBryan RPY(rad): " << imu.tait_bryan.transpose()
            << std::setprecision(COUT_PRECISION) /* set precision */
            << "\n- heading (rad): " << imu.heading
            << "\n- Temp (C): " << imu.temp << std::endl;
}
}  // namespace core::utils
#endif  // CORE_UTILS_DATA_HPP