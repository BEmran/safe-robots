// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_DATA_STRUCT_HPP_
#define CORE_UTILS_DATA_STRUCT_HPP_

#include <iostream>
#include <string>
#include <vector>

#include "core/utils/data_struct_template.hpp"
#include "core/math/math.hpp"

/* data structure labels */

namespace core::utils {

/// @brief quaternion label
const char* QUAT_LABEL = "quat";
/// @brief roll-pitch-yaw label
const char* RPY_LABEL = "rpy";
/// @brief accelerometer label
const char* ACCEL_LABEL = "accel";
/// @brief gyroscope label
const char* GYRO_LABEL = "gyro";
/// @brief magnetometer label
const char* MAG_LABEL = "mag";
/// @brief heading angle label
const char* HEADING_LABEL = "heading";
/// @brief temperature label
const char* TEMP_LABEL = "temp";
/// @brief imu label
const char* IMU_LABEL = "imu";

/* Define General Data Struct types */
using Vec3DataStruct = DataStruct<Vec3>;
using QuatDataStruct = DataStruct<Quat>;
using MathTypeDataStruct = DataStruct<MATH_TYPE>;
using AccelDataStruct = LabelDataStruct<Vec3, ACCEL_LABEL>;
using GyroDataStruct = LabelDataStruct<Vec3, GYRO_LABEL>;
using MagDataStruct = LabelDataStruct<Vec3, MAG_LABEL>;
using TemperatureDataStruct = LabelDataStruct<MATH_TYPE, TEMP_LABEL>;
using HeadingDataStruct = LabelDataStruct<MATH_TYPE, HEADING_LABEL>;

// Math_TYPE Partial Specialization -------------------------------------------
template <>
DataStruct<MATH_TYPE>::DataStruct(const std::string& label)
  : data_{0}, label_{label} {
}

template <>
void DataStruct<MATH_TYPE>::Clear() {
  data_ = 0;
}

template <>
std::string DataStruct<MATH_TYPE>::ToString() const {
  return std::to_string(data_);
}

// Vec3 Partial Specialization ------------------------------------------------
template <>
DataStruct<Vec3>::DataStruct(const std::string& label)
  : data_(Vec3::Zero()), label_(label) {
}

template <>
void DataStruct<Vec3>::Clear() {
  data_.setZero();
}

template <>
std::string DataStruct<Vec3>::Header() const {
  const auto header = label_ + ".x, " + label_ + ".y, " + label_ + ".z";
  return header;
}

template <>
std::string DataStruct<Vec3>::ToString() const {
  std::stringstream ss;
  ss << data_.format(kVecFmtSimple);
  return ss.str();
}

// Quat Partial Specialization ------------------------------------------------
template <>
DataStruct<Quat>::DataStruct(const std::string& label)
  : data_(Quat::Identity()), label_(label) {
}

template <>
void DataStruct<Quat>::Clear() {
  data_.setIdentity();
}

template <>
std::string DataStruct<Quat>::Header() const {
  static const auto header = label_ + "quat.w, " + label_ + "quat.x, " +
                             label_ + "quat.y, " + label_ + "quat.z";
  return header;
}

template <>
std::string DataStruct<Quat>::ToString() const {
  std::stringstream ss;
  ss << data_.w() << ", " << data_.vec().format(kVecFmtSimple);
  return ss.str();
}

// RPY Partial Specialization -------------------------------------------------
template <>
std::string DataStruct<RPY>::Header() const {
  static const auto header =
    label_ + "rpy.roll, " + label_ + "rpy.pitch, " + label_ + "rpy.yaw";
  return header;
}

template <>
std::string DataStruct<RPY>::ToString() const {
  std::stringstream ss;
  ss << std::to_string(data_.roll) << ", " << std::to_string(data_.pitch)
     << ", " << std::to_string(data_.yaw);
  return ss.str();
}

using RPYDataStruct = DataStruct<RPY>;

// Imu ------------------------------------------------------------------------
struct Imu {
  /// @brief thermometer, in units of degrees Celsius
  TemperatureDataStruct temp;
  /// @brief  fused heading filtered with gyro and accel data, same as rpy yaw
  /// in radians
  HeadingDataStruct heading;
  /// @brief accelerometer (XYZ) in units of m/s^2
  AccelDataStruct accel;
  /// @brief gyroscope (XYZ) in units of degrees/s
  GyroDataStruct gyro;
  /// @brief normalized quaternion
  MagDataStruct mag;
  /// @brief normalized quaternion
  QuatDataStruct quat;
  /// @brief  (roll pitch yaw) angles in radians
  RPYDataStruct rpy;
  /// @brief a list reference of all data to recall later
  const std::array<DataStructInterface* const, 7> array{
    &temp, &heading, &accel, &gyro, &mag, &quat, &rpy};

  /**
   * @brief Default constructor of Imu object
   *
   */
  Imu()
    : temp{}
    , heading{}
    , accel{}
    , gyro{}
    , mag{}
    , quat{}
    , rpy{}
    , array{&temp, &heading, &accel, &gyro, &mag, &quat, &rpy} {
  }
  /**
   * @brief Copy constructor of Imu object
   *
   * @param imu imu data object
   */
  Imu(const Imu& imu)
    : temp{imu.temp}
    , heading{imu.heading}
    , accel{imu.accel}
    , gyro{imu.gyro}
    , mag{imu.mag}
    , quat{imu.quat}
    , rpy{imu.rpy}
    , array{&temp, &heading, &accel, &gyro, &mag, &quat, &rpy} {
  }

  /**
   * @brief overload = operator to set the imu data
   *
   * @param imu new values
   * @return Imu& reference for the current object
   */
  Imu& operator=(const Imu& new_imu) {
    temp.Set(new_imu.temp.Get());
    heading.Set(new_imu.heading.Get());
    accel.Set(new_imu.accel.Get());
    gyro.Set(new_imu.gyro.Get());
    mag.Set(new_imu.mag.Get());
    quat.Set(new_imu.quat.Get());
    rpy.Set(new_imu.rpy.Get());
    return *this;
  }
  /**
   * @brief overload << operator to print imu data nicely
   *
   * @param os output-stream reference
   * @param imu imu data to be printed
   * @return std::ostream& updated output-stream
   */
  friend std::ostream& operator<<(std::ostream& os, const Imu& imu);
};

// IMU Partial Specialization -------------------------------------------------

using CB = std::function<std::string(DataStructInterface* const)>;

std::string ApplyOnEach(const std::array<DataStructInterface* const, 7> array,
                        CB cb) {
  const size_t size = 7;
  std::string str;
  for (size_t idx = 0; idx < size; ++idx) {
    if (idx == size - 1) {
      str += cb(array[idx]);
    } else {
      str += cb(array[idx]) + ", ";
    }
  }
  return str;
}

template <>
std::string DataStruct<Imu>::Header() const {
  auto lambda = [](DataStructInterface* const d) -> std::string {
    return d->Header();
  };
  return ApplyOnEach(data_.array, lambda);
}

template <>
std::string DataStruct<Imu>::ToString() const {
  auto lambda = [](DataStructInterface* const d) -> std::string {
    return d->ToString();
  };
  return ApplyOnEach(data_.array, lambda);
}

/* Define Custom Data Struct types */
using ImuDataStruct = LabelDataStruct<Imu, IMU_LABEL>;

}  // namespace core::utils
std::ostream& operator<<(std::ostream& os,
                         const core::utils::ImuDataStruct& imu_data);
#endif  // CORE_UTILS_DATA_STRUCT_HPP_
