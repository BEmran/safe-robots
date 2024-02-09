// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <sstream>

#include "core/utils/data_struct.hpp"
#include "utils_data.hpp"

using core::math::Quat;
using core::math::RPY;
using core::math::Vec3;
using core::utils::AccelDataStruct;
using core::utils::GyroDataStruct;
using core::utils::HeadingDataStruct;
using core::utils::ImuDataStruct;
using core::utils::MagDataStruct;
using core::utils::MathTypeDataStruct;
using core::utils::QuatDataStruct;
using core::utils::RPYDataStruct;
using core::utils::TemperatureDataStruct;
using core::utils::Vec3DataStruct;

// MathTypeDataStruct
// ------------------------------------------------------------
TEST(MathTypeDataStruct, Print) {
  MathTypeDataStruct mt;
  mt.Set(11.2F);
  const std::string msg = "[]: 11.200000";
  std::stringstream ss;
  ss << mt;
  EXPECT_EQ(msg, ss.str());
}

TEST(MathTypeDataStruct, Header) {
  EXPECT_EQ("", MathTypeDataStruct().Header());
}

TEST(MathTypeDataStruct, HeaderWithLabel) {
  const std::string msg = "label";
  EXPECT_EQ(msg, MathTypeDataStruct("label").Header());
}

TEST(MathTypeDataStruct, Construct) {
  const MathTypeDataStruct mt;
  EXPECT_TRUE(ExpectEq(0.f, mt.Get()));
}

TEST(MathTypeDataStruct, Clear) {
  MathTypeDataStruct mt;
  mt.Set(1.0);
  EXPECT_TRUE(ExpectEq(1.f, mt.Get()));
  mt.Clear();
  EXPECT_TRUE(ExpectEq(0.f, mt.Get()));
}
// TemperatureData -----------------------------------------------------------
TEST(TemperatureDataStruct, Header) {
  const std::string msg = "temp";
  EXPECT_EQ(msg, TemperatureDataStruct().Header());
}

// HeadingDataStruct -----------------------------------------------------------
TEST(HeadingDataStruct, Header) {
  const std::string msg = "heading";
  EXPECT_EQ(msg, HeadingDataStruct().Header());
}

// Vec3DataStruct
// -------------------------------------------------------------------
TEST(Vec3DataStruct, Print) {
  Vec3DataStruct vec;
  vec.Set({1.4F, 2.5F, 3.6F});
  const std::string msg = "[.x, .y, .z]: 1.4, 2.5, 3.6";
  std::stringstream ss;
  ss << vec;
  EXPECT_EQ(msg, ss.str());
}

TEST(Vec3DataStruct, Header) {
  const std::string msg = ".x, .y, .z";
  EXPECT_EQ(msg, Vec3DataStruct().Header());
}

TEST(Vec3DataStruct, HeaderWithLabel) {
  const auto label = "data";
  const std::string msg = "data.x, data.y, data.z";
  EXPECT_EQ(msg, Vec3DataStruct(label).Header());
}

TEST(Vec3DataStruct, ToString) {
  Vec3DataStruct vec;
  vec.Set(Vec3(1.4F, 2.5F, 3.6F));
  const std::string msg = "1.4, 2.5, 3.6";

  EXPECT_EQ(msg, vec.ToString());
}

TEST(Vec3DataStruct, Construct) {
  const Vec3DataStruct vec;
  EXPECT_TRUE(ExpectVec3Eq(Vec3{0.0, 0.0, 0.0}, vec.Get()));
}

TEST(Vec3DataStruct, Clear) {
  Vec3DataStruct vec;
  Vec3 expect{1.4F, 2.5F, 3.6F};
  vec.Set(expect);
  EXPECT_TRUE(ExpectVec3Eq(expect, vec.Get()));
  vec.Clear();
  EXPECT_TRUE(ExpectVec3Eq(Vec3{0.0, 0.0, 0.0}, vec.Get()));
}

// AccelData -------------------------------------------------------------------
TEST(AccelDataStruct, Header) {
  const std::string msg = "accel.x, accel.y, accel.z";
  EXPECT_EQ(msg, AccelDataStruct().Header());
}

// GyroData -------------------------------------------------------------------
TEST(GyroDataStruct, Header) {
  const std::string msg = "gyro.x, gyro.y, gyro.z";
  EXPECT_EQ(msg, GyroDataStruct().Header());
}

// MagData -------------------------------------------------------------------
TEST(MagDataStruct, Header) {
  const std::string msg = "mag.x, mag.y, mag.z";
  EXPECT_EQ(msg, MagDataStruct().Header());
}

// QuatDataStruct
// -------------------------------------------------------------------
TEST(QuatDataStruct, Print) {
  QuatDataStruct quat;
  quat.Set(Quat(4.7F, 1.4F, 2.5F, 3.6F));
  const std::string msg =
    "[quat.w, quat.x, quat.y, quat.z]: 4.7, 1.4, 2.5, "
    "3.6";
  std::stringstream ss;
  ss << quat;
  EXPECT_EQ(msg, ss.str());
}

TEST(QuatDataStruct, Header) {
  const std::string msg = "quat.w, quat.x, quat.y, quat.z";
  EXPECT_EQ(msg, QuatDataStruct().Header());
}

TEST(QuatDataStruct, ToString) {
  QuatDataStruct quat;
  quat.Set(Quat(4.7F, 1.4F, 2.5F, 3.6F));
  const std::string msg = "4.7, 1.4, 2.5, 3.6";
  EXPECT_EQ(msg, quat.ToString());
}

// RPYData -------------------------------------------------------------------
TEST(RPYDataStruct, Print) {
  RPYDataStruct rpy;
  rpy.Set({7.4F, 8.5F, 9.6F});
  const std::string msg =
    "[rpy.roll, rpy.pitch, rpy.yaw]: 7.400000, 8.500000, 9.600000";
  std::stringstream ss;
  ss << rpy;
  EXPECT_EQ(msg, ss.str());
}

TEST(RPYDataStruct, Header) {
  const std::string msg = "rpy.roll, rpy.pitch, rpy.yaw";
  EXPECT_EQ(msg, RPYDataStruct().Header());
}

TEST(RPYDataStruct, ToString) {
  RPYDataStruct rpy;
  rpy.Set({7.4F, 8.5F, 9.6F});
  const std::string msg = "7.400000, 8.500000, 9.600000";
  EXPECT_EQ(msg, rpy.ToString());
}

// ImuData --------------------------------------------------------------------
TEST(ImuDataStruct, Construct) {
  const ImuDataStruct imu;
  // ExpectImuData(ImuDataStruct(), imu);
}

TEST(ImuDataStruct, Clear) {
  ImuDataStruct imu;

  imu.Get().accel.Get().x() = 1.F;
  imu.Get().gyro.Set(Vec3(4.F, 5.F, 6.F));
  imu.Get().mag.Get() = Vec3(9.F, 8.F, 7.F);
  imu.Get().rpy.Get().yaw = 1.F;

  EXPECT_FLOAT_EQ(1, imu.Get().accel.Get().x());
  EXPECT_TRUE(ExpectVec3Eq(Vec3(4.F, 5.F, 6.F), imu.Get().gyro.Get()));
  EXPECT_TRUE(ExpectVec3Eq(Vec3(9.F, 8.F, 7.F), imu.Get().mag.Get()));
  EXPECT_FLOAT_EQ(1, imu.Get().rpy.Get().yaw);

  imu.Clear();

  EXPECT_TRUE(ExpectVec3Eq(Vec3::Zero(), imu.Get().accel.Get()));
  EXPECT_TRUE(ExpectVec3Eq(Vec3::Zero(), imu.Get().gyro.Get()));
  EXPECT_TRUE(ExpectVec3Eq(Vec3::Zero(), imu.Get().mag.Get()));
}

// TEST(ImuDataStruct, Print) {
//   ImuDataStruct imu;
//   // imu.accel.data = Vec3(1.4F, 2.5F, 3.6F);
//   // imu.gyro.data = Vec3(4.4F, 5.5F, 6.6F);
//   // imu.mag.data = Vec3(4.1F, 5.2F, 6.3F);
//   // imu.quat.data.w() = 4.7F;
//   // imu.quat.data.vec() = Vec3(1.4F, 2.5F, 3.6F);
//   // imu.rpy = RPY(7.4F, 8.5F, 9.6F);
//   // imu.heading.value = 10.1;
//   // imu.temp.value = 11.2;
//   const std::string msg =
//     "IMU:\n\t- Accel XYZ(m/s^2): [1.4, 2.5, 3.6]\n\t- Gyro  XYZ(rad/s): "
//     "[4.4, 5.5, 6.6]\n\t- Mag Field XYZ(uT): [4.1, 5.2, 6.3]\n\t- Quat WXYZ:
//     " "ang= 4.7, [1.4, 2.5, 3.6]\n\t- RPY(rad): [7.4, 8.5, 9.6]\n\t- "
//     "heading (rad): 10.1\n\t- Temp (C): 11.2";
//   std::stringstream ss;
//   ss << imu.Get();
//   EXPECT_EQ(msg, ss.str());
// }

TEST(ImuDataStruct, Header) {
  ImuDataStruct imu;
  const std::string msg =
    "temp, heading, accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, mag.x, "
    "mag.y, mag.z, quat.w, quat.x, quat.y, quat.z, rpy.roll, rpy.pitch, "
    "rpy.yaw";
  EXPECT_EQ(msg, imu.Header());
}

// TEST(ImuData, ToString) {
//   ImuData imu;
//   imu.accel.data = Vec3(1.4F, 2.5F, 3.6F);
//   imu.gyro.data = Vec3(4.4F, 5.5F, 6.6F);
//   imu.mag.data = Vec3(4.1F, 5.2F, 6.3F);
//   imu.quat.data.w() = 4.7F;
//   imu.quat.data.vec() = Vec3(1.4F, 2.5F, 3.6F);
//   imu.tait_bryan.data = Vec3(7.4F, 8.5F, 9.6F);
//   imu.heading.value = 10.1;
//   imu.temp.value = 11.2;
//   const std::string msg =
//     "1.4, 2.5, 3.6, 4.4, 5.5, 6.6, 4.1, 5.2, 6.3, 4.7, 1.4, 2.5, 3.6, 7.4, "
//     "8.5, 9.6, 10.100000, 11.200000, ";
//   EXPECT_EQ(msg, imu.ToString());
// }
