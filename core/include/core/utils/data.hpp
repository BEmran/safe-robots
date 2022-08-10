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
struct Data
{
 public:
  virtual ~Data(){};
  virtual void Clear() = 0;
  virtual void Print() = 0;
};

struct AdcData : public Data
{
  Vec3 values;

  void Print() override
  {
    std::cout << "Adc data: " << std::setprecision(10);
    for (int i = 0; i < 3; i++)
    {
      std::cout << "[" << i << ": " << values(i);
    }
    std::cout << "]" << std::endl;
  }

  void Clear() override
  {
    values.setZero();
  }
};

struct BarData : public Data
{
  double value = 0.0;
  void Print() override
  {
    std::cout << "Barometer data: " << std::setprecision(10) << value
              << std::endl;
  }
  void Clear() override
  {
    value = 0.0;
  }
};

struct GpsData : public Data
{
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;

  void Print() override
  {
    std::cout << "GPS data: " << std::setprecision(10)
              << "\n- latitude: " << lat  /* latitude */
              << "\n- longitude: " << lon /* longitude */
              << "\n- altitude: " << alt  /* altitude */
              << std::endl;
  }

  void Clear() override
  {
    lat = 0.0;
    lon = 0.0;
    alt = 0.0;
  }
};

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

  /**
   * @brief print imu data details
   *
   */
  void Print() override
  {
    std::cout << "IMU data: "
              << "\n- Accel XYZ(m/s^2): " << VecToString(accel)
              << "\n- Gyro  XYZ(rad/s): " << VecToString(gyro)
              << "\n- Mag Field XYZ(uT): " << VecToString(mag)
              << "\n- quat  WXYZ: " << QuatToString(quat)
              << "\n- TaitBryan RPY(rad): " << VecToString(tait_bryan)
              << std::setprecision(10) << "\n- heading (rad): " << heading
              << "\n- Temp (C): " << temp << std::endl;
  }

  void Clear() override
  {
    temp = 0.0;
    heading = 0.0;
    Zero(accel);
    Zero(gyro);
    Zero(mag);
    Zero(tait_bryan);
    Identity(quat);
  }
};

}  // namespace core::utils
#endif  // CORE_UTILS_DATA_HPP