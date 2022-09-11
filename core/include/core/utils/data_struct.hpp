// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_DATA_STRUCT_HPP_
#define CORE_UTILS_DATA_STRUCT_HPP_

// #include <iomanip>
// #include <iostream>
#include <string>
#include <vector>

#include "core/utils/math.hpp"

namespace core::utils {

struct Gps {
  double lat;
  double lon;
  double alt;
  Gps() : Gps(0.0, 0.0, 0.0) {
  }
  Gps(double lat_, double lon_, double alt_) : lat(lat_), lon(lon_), alt(alt_) {
  }
};

struct RPY {
  double roll;
  double pitch;
  double yaw;
  RPY() : RPY(0.0, 0.0, 0.0) {
  }
  RPY(double r, double p, double y) : roll(r), pitch(p), yaw(y) {
  }
};

struct DataStructInterface {
  virtual ~DataStructInterface() = default;
  virtual void Clear() = 0;
  virtual std::string Header() const = 0;
  virtual std::string ToString() const = 0;
  virtual void Set(DataStructInterface* d) = 0;
};

template <class T>
struct DataStruct : public DataStructInterface {
 public:
  explicit DataStruct(const std::string& label = "");
  ~DataStruct() = default;
  void Clear() override;
  T Get() const {
    return data_;
  }
  void Set(const T& new_data) {
    data_ = new_data;
  }
  std::string Header() const override;
  std::string ToString() const override;
  void Set(DataStructInterface* dsi) override {
    if (auto ds = dynamic_cast<DataStruct<T>*>(dsi); ds != nullptr) {
      Set(ds->Get());  // safe to call
    } else {
      throw std::bad_cast();
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const DataStruct<T>& ds) {
    return os << "[" << ds.Header() << "]: " << ds.ToString();
  }

 private:
  T data_;
  std::string label_;
};

// Math_TYPE ------------------------------------------------------------------
template <>
DataStruct<MATH_TYPE>::DataStruct(const std::string& label)
  : data_(0), label_(label) {
}

template <>
void DataStruct<MATH_TYPE>::Clear() {
  data_ = 0;
}

template <>
std::string DataStruct<MATH_TYPE>::Header() const {
  return label_;
}

template <>
std::string DataStruct<MATH_TYPE>::ToString() const {
  return std::to_string(data_);
}

// Vec3 -----------------------------------------------------------------------
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
  static const auto header = label_ + ":x, " + label_ + ":y, " + label_ + ":z";
  return header;
}

template <>
std::string DataStruct<Vec3>::ToString() const {
  std::stringstream ss;
  ss << data_.format(kVecFmtSimple);
  return ss.str();
}

// Quat -----------------------------------------------------------------------
template <>
DataStruct<Quat>::DataStruct(const std::string& label)
  : data_(Quat::Identity()), label_("quat") {
}

template <>
void DataStruct<Quat>::Clear() {
  data_.setIdentity();
}

template <>
std::string DataStruct<Quat>::Header() const {
  return "quat:w, quat:x, quat:y, quat:z";
}

template <>
std::string DataStruct<Quat>::ToString() const {
  std::stringstream ss;
  ss << data_.w() << ", " << data_.vec().format(kVecFmtSimple);
  return ss.str();
}

// Gps ------------------------------------------------------------------------
template <>
DataStruct<Gps>::DataStruct(const std::string& label)
  : data_(Gps()), label_("gps") {
}

template <>
void DataStruct<Gps>::Clear() {
  data_ = Gps();
}

template <>
std::string DataStruct<Gps>::Header() const {
  return "gps:lat, gps:long, gps:alt";
}

template <>
std::string DataStruct<Gps>::ToString() const {
  std::stringstream ss;
  ss << std::to_string(data_.lat) << ", " << std::to_string(data_.lon) << ", "
     << std::to_string(data_.alt);
  return ss.str();
}

// RPY ------------------------------------------------------------------------
template <>
DataStruct<RPY>::DataStruct(const std::string& label)
  : data_(RPY()), label_("rpy") {
}

template <>
void DataStruct<RPY>::Clear() {
  data_ = RPY();
}

template <>
std::string DataStruct<RPY>::Header() const {
  return "rpy:roll, rpy:pitch, rpy:yaw";
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
using GpsDataStruct = DataStruct<Gps>;
using MathTypeDataStruct = DataStruct<MATH_TYPE>;
using RPYDataStruct = DataStruct<RPY>;

class AccelDataStruct : public Vec3DataStruct {
 public:
  AccelDataStruct() : DataStruct("accel") {
  }
};

class GyroDataStruct : public Vec3DataStruct {
 public:
  GyroDataStruct() : DataStruct("gyro") {
  }
};

class MagDataStruct : public Vec3DataStruct {
 public:
  MagDataStruct() : DataStruct("mag") {
  }
};

class TemperatureDataStruct : public MathTypeDataStruct {
 public:
  TemperatureDataStruct() : DataStruct("temp") {
  }
};

class HeadingDataStruct : public MathTypeDataStruct {
 public:
  HeadingDataStruct() : DataStruct("heading") {
  }
};

struct Imu {
  TemperatureDataStruct temp;  ///< thermometer, in units of degrees Celsius
  HeadingDataStruct heading;   ///< fused heading filtered with gyro and accel
                               ///< data, same as Tait-Bryan yaw in radians
  AccelDataStruct accel;       ///< accelerometer (XYZ) in units of m/s^2
  GyroDataStruct gyro;         ///< gyroscope (XYZ) in units of degrees/s
  MagDataStruct mag;           ///< magnetometer (XYZ) in units of uT
  QuatDataStruct quat;         ///< normalized quaternion
  RPYDataStruct tait_bryan;  ///< Tait-Bryan angles (roll pitch yaw) in radians
  std::array<DataStructInterface* const, 7> array = {
    &temp, &heading, &accel, &gyro, &mag, &quat, &tait_bryan};

  Imu() {
  }
  void operator=(const Imu& imu) {
    temp.Set(imu.temp.Get());
    heading.Set(imu.heading.Get());
    accel.Set(imu.accel.Get());
    gyro.Set(imu.gyro.Get());
    mag.Set(imu.mag.Get());
    quat.Set(imu.quat.Get());
    tait_bryan.Set(imu.tait_bryan.Get());
  }
};

// Imu ------------------------------------------------------------------------
template <>
DataStruct<Imu>::DataStruct(const std::string& label)
  : data_(Imu()), label_("imu") {
}

template <>
void DataStruct<Imu>::Clear() {
  std::for_each(data_.array.begin(), data_.array.end(),
                [](DataStructInterface* const ptr) { ptr->Clear(); });
}

template <>
std::string DataStruct<Imu>::Header() const {
  std::stringstream ss;
  std::for_each(
    data_.array.begin(), data_.array.end() - 1,
    [&ss](DataStructInterface* const ptr) { ss << ptr->Header() << ", "; });
  ss << data_.array.back()->Header();
  return ss.str();
}

template <>
std::string DataStruct<Imu>::ToString() const {
  std::stringstream ss;
  std::for_each(
    data_.array.begin(), data_.array.end() - 1,
    [&ss](DataStructInterface* const ptr) { ss << ptr->ToString() << ", "; });
  ss << data_.array.back()->ToString();
  return ss.str();
}

using ImuDataStruct = DataStruct<Imu>;

}  // namespace core::utils

namespace cu = core::utils;

/* print data details */

std::ostream& operator<<(std::ostream& os,
                         const cu::DataStructInterface* const dsi);

std::ostream& operator<<(std::ostream& os, const cu::Imu& imu);

#endif  // CORE_UTILS_DATA_STRUCT_HPP_
