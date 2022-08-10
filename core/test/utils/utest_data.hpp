#ifndef TEST_UTEST_DATA_HPP
#define TEST_UTEST_DATA_HPP

#include "core/utils/data.hpp"

#include "gtest/gtest.h"

using core::utils::ImuData;
using core::utils::Quat;
using core::utils::Vec3;
using core::utils::GpsData;

void ExpectVec3Eq(const Vec3& v1, const Vec3& v2)
{
  EXPECT_FLOAT_EQ(v1.x(), v2.x());
  EXPECT_FLOAT_EQ(v1.y(), v2.y());
  EXPECT_FLOAT_EQ(v1.z(), v2.z());
}

void ExpectQuatEq(const Quat& q1, const Quat& q2)
{
  ExpectVec3Eq(q1.vec(), q2.vec());
  EXPECT_FLOAT_EQ(q1.w(), q2.w());
}

void ExpectImuData(const ImuData& d1, const ImuData& d2)
{
  EXPECT_DOUBLE_EQ(d1.heading, d2.heading);
  ExpectVec3Eq(d1.tait_bryan, d2.tait_bryan);
  ExpectVec3Eq(d1.accel, d2.accel);
  ExpectVec3Eq(d1.gyro, d2.gyro);
  ExpectVec3Eq(d1.mag, d2.mag);
  ExpectQuatEq(d1.quat, d2.quat);
  EXPECT_DOUBLE_EQ(d1.temp, d2.temp);
}

void ExpectGPSData(const double lat, const double lon, const double alt, const GpsData& gps)
{
  EXPECT_DOUBLE_EQ(lat, gps.lat);
  EXPECT_DOUBLE_EQ(lon, gps.lon);
  EXPECT_DOUBLE_EQ(alt, gps.alt);
}
#endif  // TEST_UTEST_DATA_HPP