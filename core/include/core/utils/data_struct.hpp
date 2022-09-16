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
  virtual void Set(DataStructInterface* d) = 0;
};

template <class T>
struct DataStruct : public DataStructInterface {
 public:
  explicit DataStruct(const std::string& label = "") : label_{label} {};
  explicit DataStruct(const DataStruct& ds)
    : data_{ds.Get()}, label_{ds.Label()} {
  }
  ~DataStruct() = default;
  T Get() const {
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
  T data_;
  std::string label_;
};

template <class TYPE, auto& LABEL>
struct LabelDataStruct : public DataStruct<TYPE> {
  LabelDataStruct() : DataStruct<TYPE>(LABEL) {
  }
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

// template <>
// std::string DataStruct<MATH_TYPE>::Header() const {
//   return label_;
// }

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

// Gps ------------------------------------------------------------------------
// template <>
// DataStruct<Gps>::DataStruct(const std::string& label)
//   : data_(Gps()), label_("gps") {
// }

// template <>
// void DataStruct<Gps>::Clear() {
//   data_ = Gps();
// }

template <>
std::string DataStruct<Gps>::Header() const {
  static const auto header =
    label_ + "gps.lat, " + label_ + "gps.long, " + label_ + "gps.alt";
  return header;
}

template <>
std::string DataStruct<Gps>::ToString() const {
  std::stringstream ss;
  ss << std::to_string(data_.lat) << ", " << std::to_string(data_.lon) << ", "
     << std::to_string(data_.alt);
  return ss.str();
}

// RPY ------------------------------------------------------------------------
// template <>
// DataStruct<RPY>::DataStruct(const std::string& label)
//   : data_(RPY()), label_("rpy") {
// }

// template <>
// void DataStruct<RPY>::Clear() {
//   data_ = RPY();
// }

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
using GpsDataStruct = DataStruct<Gps>;
using MathTypeDataStruct = DataStruct<MATH_TYPE>;
using RPYDataStruct = DataStruct<RPY>;

using AccelDataStruct = LabelDataStruct<Vec3, ACCEL_LABEL>;
using GyroDataStruct = LabelDataStruct<Vec3, GYRO_LABEL>;
using MagDataStruct = LabelDataStruct<Vec3, MAG_LABEL>;
using TemperatureDataStruct = LabelDataStruct<MATH_TYPE, TEMP_LABEL>;
using HeadingDataStruct = LabelDataStruct<MATH_TYPE, HEADING_LABEL>;

// class AccelDataStruct : public Vec3DataStruct {
//  public:
//   AccelDataStruct() : Vec3DataStruct("accel") {
//   }
// };

// class GyroDataStruct : public Vec3DataStruct {
//  public:
//   GyroDataStruct() : Vec3DataStruct("gyro") {
//   }
// };

// class MagDataStruct : public Vec3DataStruct {
//  public:
//   MagDataStruct() : Vec3DataStruct("mag") {
//   }
// };

// class TemperatureDataStruct : public MathTypeDataStruct {
//  public:
//   TemperatureDataStruct() : MathTypeDataStruct("temp") {
//   }
// };

// class HeadingDataStruct : public MathTypeDataStruct {
//  public:
//   HeadingDataStruct() : MathTypeDataStruct("heading") {
//   }
// };

struct Imu {
  TemperatureDataStruct temp;  ///< thermometer, in units of degrees Celsius
  HeadingDataStruct heading;   ///< fused heading filtered with gyro and accel
                               ///< data, same as Tait-Bryan yaw in radians
  AccelDataStruct accel;       ///< accelerometer (XYZ) in units of m/s^2
  GyroDataStruct gyro;         ///< gyroscope (XYZ) in units of degrees/s
  MagDataStruct mag;           ///< magnetometer (XYZ) in units of uT
  QuatDataStruct quat;         ///< normalized quaternion
  RPYDataStruct tait_bryan;  ///< Tait-Bryan angles (roll pitch yaw) in radians
  const std::array<DataStructInterface* const, 7> array = {
    &temp, &heading, &accel, &gyro, &mag, &quat, &tait_bryan};

  Imu() {
  }

  Imu& operator=(const Imu& imu) {
    temp.Set(imu.temp.Get());
    heading.Set(imu.heading.Get());
    accel.Set(imu.accel.Get());
    gyro.Set(imu.gyro.Get());
    mag.Set(imu.mag.Get());
    quat.Set(imu.quat.Get());
    tait_bryan.Set(imu.tait_bryan.Get());
    return *this;
  }

  // Imu(const Imu& imu) :
  //   temp{imu.temp.Get()},
  //   heading{imu.heading.Get()},
  //   accel{imu.accel.Get()},
  //   gyro{imu.gyro.Get()},
  //   mag{imu.mag.Get()},
  //   quat{imu.quat.Get()},
  //   tait_bryan{imu.tait_bryan.Get()}{
  // }
  Imu(const Imu& imu)
    : temp{imu.temp}
    , heading{imu.heading}
    , accel{imu.accel}
    , gyro{imu.gyro}
    , mag{imu.mag}
    , quat{imu.quat}
    , tait_bryan{imu.tait_bryan}
    , array{&temp, &heading, &accel, &gyro, &mag, &quat, &tait_bryan} {
  }
};

// Imu ------------------------------------------------------------------------
// template <>
// DataStruct<Imu>::DataStruct(const std::string& label)
//   : data_(Imu()), label_("imu") {
// }

// template <>
// void DataStruct<Imu>::Clear() {
//   std::for_each(data_.array.begin(), data_.array.end(),
//                 [](DataStructInterface* const ptr) { ptr->Clear(); });
// }

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

using ImuDataStruct = LabelDataStruct<Imu, IMU_LABEL>;

}  // namespace core::utils

namespace cu = core::utils;

/* print data details */

std::ostream& operator<<(std::ostream& os,
                         const cu::DataStructInterface* const dsi);

std::ostream& operator<<(std::ostream& os, const cu::Imu& imu);

#endif  // CORE_UTILS_DATA_STRUCT_HPP_
