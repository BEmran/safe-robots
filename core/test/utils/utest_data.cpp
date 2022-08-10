#include <gtest/gtest.h>

#include "utest_data.hpp"
#include <sstream>

using namespace core::utils;

// AdcData --------------------------------------------------------------------

TEST(AdcData, Consruct)
{
  const AdcData adc;
  ExpectVec3Eq(Vec3::Zero(), adc.values);
}

TEST(AdcData, Clear)
{
  AdcData adc;
  Vec3 vec(1.4F, 2.5F, 3.6F);
  adc.values = vec;
  ExpectVec3Eq(vec, adc.values);
  adc.Clear();
  ExpectVec3Eq(Vec3::Zero(), adc.values);
}

TEST(AdcData, Print)
{
  AdcData adc;
  adc.values = Vec3(1.4F, 2.5F, 3.6F);
  std::stringstream ss;
  ss << adc;
  EXPECT_EQ("ADC data: [1.4, 2.5, 3.6]\n", ss.str());
}

// BarData --------------------------------------------------------------------

TEST(BarData, Consruct)
{
  const BarData bar;
  EXPECT_DOUBLE_EQ(0, bar.value);
}

TEST(BarData, Clear)
{
  BarData bar;
  bar.value = 1.2;
  EXPECT_DOUBLE_EQ(1.2, bar.value);
  bar.Clear();
  EXPECT_DOUBLE_EQ(0, bar.value);
}

TEST(BarData, Print)
{
  BarData bar;
  bar.value = 1.2;
  std::stringstream ss;
  ss << bar;
  EXPECT_EQ("Barometer data: 1.2\n", ss.str());
}

// GpsData --------------------------------------------------------------------

TEST(GpsData, Consruct)
{
  const GpsData gps;
  ExpectGPSData(0, 0, 0, gps);
}

TEST(GpsData, Clear)
{
  GpsData gps;
  gps.lat = 1.2;
  EXPECT_DOUBLE_EQ(1.2, gps.lat);
  gps.Clear();
  EXPECT_DOUBLE_EQ(0, gps.lat);
}

TEST(GpsData, Print)
{
  GpsData gps;
  gps.lat = 1.4;
  gps.lon = 2.5;
  gps.alt = 3.6;
  std::stringstream ss;
  ss << gps;
  EXPECT_EQ("GPS data: \n- Lat: 1.4\n- Lon: 2.5\n- Alt: 3.6\n", ss.str());
}

// ImuData --------------------------------------------------------------------

TEST(ImuData, Consruct)
{
  const ImuData imu;
  ExpectImuData(ImuData(), imu);
}

TEST(ImuData, Clear)
{
  ImuData imu;
  imu.gyro.x() = 1;
  EXPECT_FLOAT_EQ(1, imu.gyro.x());
  imu.Clear();
  EXPECT_FLOAT_EQ(0, imu.gyro.x());
}

TEST(ImuData, Print)
{
  ImuData imu;
  imu.accel = Vec3(1.4F, 2.5F, 3.6F);
  imu.gyro = Vec3(4.4F, 5.5F, 6.6F);
  imu.mag = Vec3(4.1F, 5.2F, 6.3F);
  imu.quat.w() = 4.7F;
  imu.quat.vec() = Vec3(1.4F, 2.5F, 3.6F);
  imu.tait_bryan = Vec3(7.4F, 8.5F, 9.6F);
  imu.heading = 10.1;
  imu.temp = 11.2;
  const std::string msg =
      "IMU data:\n- Accel XYZ(m/s^2): [1.4, 2.5, 3.6]\n- Gyro  XYZ(rad/s): "
      "[4.4, 5.5, 6.6]\n- Mag Field XYZ(uT): [4.1, 5.2, 6.3]\n- Quat WXYZ: ang "
      "= 4.7, [1.4, 2.5, 3.6]\n- TaitBryan RPY(rad): [7.4, 8.5, 9.6]\n- "
      "heading (rad): 10.1\n- Temp (C): 11.2\n";
  std::stringstream ss;
  ss << imu;
  EXPECT_EQ(msg, ss.str());
}
