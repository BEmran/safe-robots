// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <sstream>

#include "utest/utils_data.hpp"

using core::utils::AccelData;
using core::utils::AdcData;
using core::utils::BarData;
using core::utils::GpsData;
using core::utils::GyroData;
using core::utils::HeadingData;
using core::utils::ImuData;
using core::utils::MagData;
using core::utils::QuatData;
using core::utils::RPYData;
using core::utils::TemperatureData;
using core::utils::Vec3;
using core::utils::Vec3Data;
// AdcData --------------------------------------------------------------------

TEST(AdcData, Construct) {
  const AdcData adc;
  EXPECT_TRUE(ExpectVec3Eq(Vec3::Zero(), adc.values));
}

TEST(AdcData, Clear) {
  AdcData adc;
  Vec3 vec(1.4F, 2.5F, 3.6F);
  adc.values = vec;
  ExpectVec3Eq(vec, adc.values);
  adc.Clear();
  EXPECT_TRUE(ExpectVec3Eq(Vec3::Zero(), adc.values));
}

TEST(AdcData, Print) {
  AdcData adc;
  adc.values = Vec3(1.4F, 2.5F, 3.6F);
  std::stringstream ss;
  ss << adc;
  EXPECT_EQ("ADC data: [1.4, 2.5, 3.6]", ss.str());
}

TEST(AdcData, Header) {
  const std::string msg = "adc:0, adc:1, adc:2";
  EXPECT_EQ(msg, AdcData().Header());
}

TEST(AdcData, ToString) {
  AdcData adc;
  adc.values = Vec3(1.4F, 2.5F, 3.6F);
  const std::string msg = "1.4, 2.5, 3.6";
  EXPECT_EQ(msg, adc.ToString());
}

// BarData --------------------------------------------------------------------
TEST(BarData, Construct) {
  const BarData bar;
  EXPECT_DOUBLE_EQ(0, bar.value);
}

TEST(BarData, Clear) {
  BarData bar;
  bar.value = 1.2;
  EXPECT_DOUBLE_EQ(1.2, bar.value);
  bar.Clear();
  EXPECT_DOUBLE_EQ(0, bar.value);
}

TEST(BarData, Print) {
  BarData bar;
  bar.value = 1.2;
  std::stringstream ss;
  ss << bar;
  EXPECT_EQ("Barometer data: 1.2", ss.str());
}

TEST(BarData, Header) {
  const std::string msg = "bar";
  EXPECT_EQ(msg, BarData().Header());
}

TEST(BarData, ToString) {
  BarData bar;
  bar.value = 1.2;
  const std::string msg = "1.200000";
  EXPECT_EQ(msg, bar.ToString());
}

// GpsData --------------------------------------------------------------------
TEST(GpsData, Construct) {
  const GpsData gps;
  EXPECT_TRUE(ExpectGPSData(0, 0, 0, gps));
}

TEST(GpsData, Clear) {
  GpsData gps;
  gps.lat = 1.2;
  EXPECT_DOUBLE_EQ(1.2, gps.lat);
  gps.Clear();
  EXPECT_DOUBLE_EQ(0, gps.lat);
}

TEST(GpsData, Print) {
  GpsData gps;
  gps.lat = 1.4;
  gps.lon = 2.5;
  gps.alt = 3.6;
  std::stringstream ss;
  ss << gps;
  EXPECT_EQ("GPS data: \n- Lat: 1.4\n- Lon: 2.5\n- Alt: 3.6\n", ss.str());
}

TEST(GpsData, Header) {
  const std::string msg = "gps:lat, gps:long, gps:alt";
  EXPECT_EQ(msg, GpsData().Header());
}

TEST(GpsData, ToString) {
  GpsData gps;
  gps.lat = 1.4;
  gps.lon = 2.5;
  gps.alt = 3.6;
  const std::string msg = "1.400000, 2.500000, 3.600000";
  EXPECT_EQ(msg, gps.ToString());
}

// Vec3Data -------------------------------------------------------------------
TEST(Vec3Data, Print) {
  Vec3Data vec;
  vec.data = Vec3(1.4F, 2.5F, 3.6F);
  const std::string msg = "[1.4, 2.5, 3.6]";
  std::stringstream ss;
  ss << vec;
  EXPECT_EQ(msg, ss.str());
}

TEST(Vec3Data, Header) {
  const std::string msg = "data:x, data:y, data:z";
  EXPECT_EQ(msg, Vec3Data().Header());
}

TEST(Vec3Data, ToString) {
  Vec3Data vec;
  vec.data = Vec3(1.4F, 2.5F, 3.6F);
  const std::string msg = "1.4, 2.5, 3.6";

  EXPECT_EQ(msg, vec.ToString());
}

// AccelData -------------------------------------------------------------------
TEST(AccelData, Print) {
  AccelData accel;
  accel.data = Vec3(1.4F, 2.5F, 3.6F);
  const std::string msg = "Accel XYZ(m/s^2): [1.4, 2.5, 3.6]";
  std::stringstream ss;
  ss << accel;
  EXPECT_EQ(msg, ss.str());
}

TEST(AccelData, Header) {
  const std::string msg = "accel:x, accel:y, accel:z";
  EXPECT_EQ(msg, AccelData().Header());
}

TEST(AccelData, ToString) {
  AccelData accel;
  accel.data = Vec3(1.4F, 2.5F, 3.6F);
  const std::string msg = "1.4, 2.5, 3.6";
  EXPECT_EQ(msg, accel.ToString());
}

// GyroData -------------------------------------------------------------------
TEST(GyroData, Print) {
  GyroData gyro;
  gyro.data = Vec3(4.4F, 5.5F, 6.6F);
  const std::string msg = "Gyro  XYZ(rad/s): [4.4, 5.5, 6.6]";
  std::stringstream ss;
  ss << gyro;
  EXPECT_EQ(msg, ss.str());
}

TEST(GyroData, Header) {
  const std::string msg = "gyro:x, gyro:y, gyro:z";
  EXPECT_EQ(msg, GyroData().Header());
}

TEST(GyroData, ToString) {
  GyroData gyro;
  gyro.data = Vec3(4.4F, 5.5F, 6.6F);
  const std::string msg = "4.4, 5.5, 6.6";
  EXPECT_EQ(msg, gyro.ToString());
}

// GyroData -------------------------------------------------------------------
TEST(MagData, Print) {
  MagData mag;
  mag.data = Vec3(4.1F, 5.2F, 6.3F);
  const std::string msg = "Mag Field XYZ(uT): [4.1, 5.2, 6.3]";
  std::stringstream ss;
  ss << mag;
  EXPECT_EQ(msg, ss.str());
}

TEST(MagData, Header) {
  const std::string msg = "mag:x, mag:y, mag:z";
  EXPECT_EQ(msg, MagData().Header());
}

TEST(MagData, ToString) {
  MagData mag;
  mag.data = Vec3(4.1F, 5.2F, 6.3F);
  const std::string msg = "4.1, 5.2, 6.3";
  EXPECT_EQ(msg, mag.ToString());
}

// QuatData -------------------------------------------------------------------
TEST(QuatData, Print) {
  QuatData quat;
  quat.data.w() = 4.7F;
  quat.data.vec() = Vec3(1.4F, 2.5F, 3.6F);
  const std::string msg = "Quat WXYZ: ang = 4.7, [1.4, 2.5, 3.6]";
  std::stringstream ss;
  ss << quat;
  EXPECT_EQ(msg, ss.str());
}

TEST(QuatData, Header) {
  const std::string msg = "quat:w, quat:x, quat:y, quat:z";
  EXPECT_EQ(msg, QuatData().Header());
}

TEST(QuatData, ToString) {
  QuatData quat;
  quat.data.w() = 4.7F;
  quat.data.vec() = Vec3(1.4F, 2.5F, 3.6F);
  const std::string msg = "4.7, 1.4, 2.5, 3.6";
  EXPECT_EQ(msg, quat.ToString());
}

// RPYData -------------------------------------------------------------------
TEST(RPYData, Print) {
  RPYData rpy;
  rpy.data = Vec3(7.4F, 8.5F, 9.6F);
  const std::string msg = "TaitBryan RPY(rad): [7.4, 8.5, 9.6]";
  std::stringstream ss;
  ss << rpy;
  EXPECT_EQ(msg, ss.str());
}

TEST(RPYData, Header) {
  const std::string msg = "roll, pitch, yaw";
  EXPECT_EQ(msg, RPYData().Header());
}

TEST(RPYData, ToString) {
  RPYData rpy;
  rpy.data = Vec3(7.4F, 8.5F, 9.6F);
  const std::string msg = "7.4, 8.5, 9.6";
  EXPECT_EQ(msg, rpy.ToString());
}

// TemperatureData ------------------------------------------------------------
TEST(TemperatureData, Print) {
  TemperatureData temp;
  temp.value = 11.2F;
  const std::string msg = "Temp (C): 11.2";
  std::stringstream ss;
  ss << temp;
  EXPECT_EQ(msg, ss.str());
}

TEST(TemperatureData, Header) {
  const std::string msg = "temp";
  EXPECT_EQ(msg, TemperatureData().Header());
}

TEST(TemperatureData, ToString) {
  TemperatureData temp;
  temp.value = 11.2F;
  const std::string msg = "11.200000";

  EXPECT_EQ(msg, temp.ToString());
}

// HeadingData ----------------------------------------------------------------
TEST(HeadingData, Print) {
  HeadingData head;
  head.value = 10.1F;
  const std::string msg = "heading (rad): 10.1";
  std::stringstream ss;
  ss << head;
  EXPECT_EQ(msg, ss.str());
}

TEST(HeadingData, Header) {
  const std::string msg = "heading";
  EXPECT_EQ(msg, HeadingData().Header());
}

TEST(HeadingData, ToString) {
  HeadingData head;
  head.value = 10.1F;
  const std::string msg = "10.100000";
  EXPECT_EQ(msg, head.ToString());
}

// ImuData --------------------------------------------------------------------
TEST(ImuData, Construct) {
  const ImuData imu;
  EXPECT_TRUE(ExpectImuData(ImuData(), imu));
}

TEST(ImuData, Clear) {
  ImuData imu;
  imu.gyro.data.x() = 1;
  EXPECT_FLOAT_EQ(1, imu.gyro.data.x());
  imu.Clear();
  EXPECT_FLOAT_EQ(0, imu.gyro.data.x());
}

TEST(ImuData, Print) {
  ImuData imu;
  imu.accel.data = Vec3(1.4F, 2.5F, 3.6F);
  imu.gyro.data = Vec3(4.4F, 5.5F, 6.6F);
  imu.mag.data = Vec3(4.1F, 5.2F, 6.3F);
  imu.quat.data.w() = 4.7F;
  imu.quat.data.vec() = Vec3(1.4F, 2.5F, 3.6F);
  imu.tait_bryan.data = Vec3(7.4F, 8.5F, 9.6F);
  imu.heading.value = 10.1;
  imu.temp.value = 11.2;
  const std::string msg =
    "IMU data:\n- Accel XYZ(m/s^2): [1.4, 2.5, 3.6]\n- Gyro  XYZ(rad/s): "
    "[4.4, 5.5, 6.6]\n- Mag Field XYZ(uT): [4.1, 5.2, 6.3]\n- Quat WXYZ: ang "
    "= 4.7, [1.4, 2.5, 3.6]\n- TaitBryan RPY(rad): [7.4, 8.5, 9.6]\n- "
    "heading (rad): 10.1\n- Temp (C): 11.2\n";
  std::stringstream ss;
  ss << imu;
  EXPECT_EQ(msg, ss.str());
}

TEST(ImuData, Header) {
  ImuData imu;
  const std::string msg =
    "accel:x, accel:y, accel:z, gyro:x, gyro:y, gyro:z, mag:x, mag:y, mag:z, "
    "quat:w, quat:x, quat:y, quat:z, roll, pitch, yaw, heading, temp";
  EXPECT_EQ(msg, imu.Header());
}

TEST(ImuData, ToString) {
  ImuData imu;
  imu.accel.data = Vec3(1.4F, 2.5F, 3.6F);
  imu.gyro.data = Vec3(4.4F, 5.5F, 6.6F);
  imu.mag.data = Vec3(4.1F, 5.2F, 6.3F);
  imu.quat.data.w() = 4.7F;
  imu.quat.data.vec() = Vec3(1.4F, 2.5F, 3.6F);
  imu.tait_bryan.data = Vec3(7.4F, 8.5F, 9.6F);
  imu.heading.value = 10.1;
  imu.temp.value = 11.2;
  const std::string msg =
    "1.4, 2.5, 3.6, 4.4, 5.5, 6.6, 4.1, 5.2, 6.3, 4.7, 1.4, 2.5, 3.6, 7.4, "
    "8.5, 9.6, 10.100000, 11.200000";
  EXPECT_EQ(msg, imu.ToString());
}
