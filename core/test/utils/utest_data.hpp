#ifndef TEST_UTEST_DATA_HPP
#define TEST_UTEST_DATA_HPP

#include "core/utils/data.hpp"

#include "gtest/gtest.h"

using core::utils::ImuData;
using core::utils::Quat;
using core::utils::Vec3;

void ExpectVec3(const Vec3& v1, const Vec3& v2)
{
  EXPECT_DOUBLE_EQ(v1.x(), v2.x());
  EXPECT_DOUBLE_EQ(v1.y(), v2.y());
  EXPECT_DOUBLE_EQ(v1.z(), v2.z());
}

void ExpectQuat(const Quat& q1, const Quat& q2)
{
  ExpectVec3(q1.vector, q2.vector);
  EXPECT_DOUBLE_EQ(q1.angle, q2.angle);
}

void ExpectImuData(const ImuData& d1, const ImuData& d2)
{
  EXPECT_DOUBLE_EQ(d1.heading, d2.heading);
  ExpectVec3(d1.tait_bryan, d2.tait_bryan);
  ExpectVec3(d1.accel, d2.accel);
  ExpectVec3(d1.gyro, d2.gyro);
  ExpectVec3(d1.mag, d2.mag);
  ExpectQuat(d1.quat, d2.quat);
  ExpectQuat(d1.temp, d2.temp);
}
#endif  // TEST_UTEST_DATA_HPP