// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_DATA_STRUCT_HPP_
#define CORE_UTILS_DATA_STRUCT_HPP_

// #include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "core/utils/math.hpp"

const char* QUAT_LABEL = "quat";
const char* GPS_LABEL = "gps";
const char* RPY_LABEL = "rpy";
const char* ACCEL_LABEL = "accel";
const char* GYRO_LABEL = "gyro";
const char* MAG_LABEL = "mag";
const char* HEADING_LABEL = "heading";
const char* TEMP_LABEL = "temp";
const char* IMU_LABEL = "imu";

namespace core::utils {

// Custom data needs to have default constructor

struct RPY {
  MATH_TYPE roll;
  MATH_TYPE pitch;
  MATH_TYPE yaw;
  RPY() : RPY(0.0, 0.0, 0.0) {
  }
  RPY(MATH_TYPE r, MATH_TYPE p, MATH_TYPE y) : roll(r), pitch(p), yaw(y) {
  }
};

struct DataStructInterface {
  virtual ~DataStructInterface() = default;
  virtual void Clear() = 0;
  virtual std::string Label() const = 0;
  virtual std::string Header() const = 0;
  virtual std::string ToString() const = 0;
  virtual std::string ToString() const = 0;
  virtual void Set(DataStructInterface* d) = 0;
};

template <class T>
struct DataStruct : public DataStructInterface {
 public:
  explicit DataStruct(const std::string& label = "")
    : data_{}, label_{label} {};

  explicit DataStruct(const DataStruct<T>& ds)
    : data_{ds.Get()}, label_{ds.Label()} {
  }

  ~DataStruct() = default;

  T Get() const {
    return data_;
  }

  T& Get() {
    return data_;
  }

  void Set(const T& new_data) {
    data_ = new_data;
  }

  void Clear() override {
    data_ = T{};
  }

  std::string Label() const override {
    return label_;
  }

  std::string Header() const override {
    return label_;
  }

  std::string ToString() const override {
    return "";
  }

  void Set(DataStructInterface* dsi_ptr) override {
    if (auto ds = dynamic_cast<DataStruct<T>*>(dsi_ptr); ds != nullptr) {
      Set(ds->Get());  // safe to call
    } else {
      throw std::bad_cast();
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const DataStruct<T>& ds) {
    return os << "[" << ds.Header() << "]: " << ds.ToString();
  }

 protected:
  T data_{};
  std::string label_{};
};

template <class TYPE, auto& LABEL>
struct LabelDataStruct : public DataStruct<TYPE> {
  LabelDataStruct() : DataStruct<TYPE>(LABEL) {
  }
};

// Math_TYPE ------------------------------------------------------------------
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

// Vec3 ----------------------------------------------------------------------
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

// Quat
// -----------------------------------------------------------------------
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

// RPY ------------------------------------------------------------------------
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

using Vec3DataStruct = DataStruct<Vec3>;
using QuatDataStruct = DataStruct<Quat>;
using MathTypeDataStruct = DataStruct<MATH_TYPE>;
using RPYDataStruct = DataStruct<RPY>;

using AccelDataStruct = LabelDataStruct<Vec3, ACCEL_LABEL>;
using GyroDataStruct = LabelDataStruct<Vec3, GYRO_LABEL>;
using MagDataStruct = LabelDataStruct<Vec3, MAG_LABEL>;
using TemperatureDataStruct = LabelDataStruct<MATH_TYPE, TEMP_LABEL>;
using HeadingDataStruct = LabelDataStruct<MATH_TYPE, HEADING_LABEL>;

// Imu ------------------------------------------------------------------------
struct Imu {
  TemperatureDataStruct temp;  ///< thermometer, in units of degrees Celsius
  HeadingDataStruct heading;   ///< fused heading filtered with gyro and accel
                               ///< data, same as rpy yaw in radians
  AccelDataStruct accel;       ///< accelerometer (XYZ) in units of m/s^2
  GyroDataStruct gyro;         ///< gyroscope (XYZ) in units of degrees/s
  MagDataStruct mag;           ///< magnetometer (XYZ) in units of uT
  QuatDataStruct quat;         ///< normalized quaternion
  RPYDataStruct rpy;           ///< (roll pitch yaw) angles in radians
  const std::array<DataStructInterface* const, 7> array{
    &temp, &heading, &accel, &gyro, &mag, &quat, &rpy};

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

  Imu& operator=(const Imu& imu) {
    temp.Set(imu.temp.Get());
    heading.Set(imu.heading.Get());
    accel.Set(imu.accel.Get());
    gyro.Set(imu.gyro.Get());
    mag.Set(imu.mag.Get());
    quat.Set(imu.quat.Get());
    rpy.Set(imu.rpy.Get());
    return *this;
  }

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
};

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

using ImuDataStruct = LabelDataStruct<Imu, IMU_LABEL>;

}  // namespace core::utils

namespace cu = core::utils;

/* print data details */

std::ostream& operator<<(std::ostream& os,
                         const cu::DataStructInterface* const dsi);

// std::ostream& operator<<(std::ostream& os, const cu::Imu& imu);

#endif  // CORE_UTILS_DATA_STRUCT_HPP_
