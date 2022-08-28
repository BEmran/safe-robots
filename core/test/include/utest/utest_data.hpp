// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_TEST_INCLUDE_UTEST_UTEST_DATA_HPP_
#define CORE_TEST_INCLUDE_UTEST_UTEST_DATA_HPP_

#include "core/utils/data.hpp"
#include "gtest/gtest.h"

using core::utils::DoubleData;
using core::utils::GpsData;
using core::utils::ImuData;
using core::utils::MATH_TYPE;
using core::utils::Quat;
using core::utils::QuatData;
using core::utils::Vec3;
using core::utils::Vec3Data;

void ExpectEq(const MATH_TYPE actual, const MATH_TYPE expect) {
  if constexpr (std::is_same<MATH_TYPE, float>::value) {
    EXPECT_FLOAT_EQ(actual, expect);
  } else {
    EXPECT_DOUBLE_EQ(actual, expect);
  }
}

void ExpectDoubleDataEq(const DoubleData& d1, const DoubleData& d2) {
  EXPECT_DOUBLE_EQ(d1.value, d2.value);
}

void ExpectVec3Eq(const Vec3& v1, const Vec3& v2) {
  ExpectEq(v1.x(), v2.x());
  ExpectEq(v1.y(), v2.y());
  ExpectEq(v1.z(), v2.z());
}

void ExpectVec3DataEq(const Vec3Data& v1, const Vec3Data& v2) {
  ExpectVec3Eq(v1.data, v2.data);
}

void ExpectQuatEq(const Quat& q1, const Quat& q2) {
  ExpectVec3Eq(q1.vec(), q2.vec());
  ExpectEq(q1.w(), q2.w());
}

void ExpectQuatDataEq(const QuatData& q1, const QuatData& q2) {
  ExpectQuatEq(q1.data, q2.data);
}

void ExpectImuData(const ImuData& d1, const ImuData& d2) {
  ExpectDoubleDataEq(d1.heading, d2.heading);
  ExpectDoubleDataEq(d1.temp, d2.temp);
  ExpectVec3DataEq(d1.tait_bryan, d2.tait_bryan);
  ExpectVec3DataEq(d1.accel, d2.accel);
  ExpectVec3DataEq(d1.gyro, d2.gyro);
  ExpectVec3DataEq(d1.mag, d2.mag);
  ExpectQuatDataEq(d1.quat, d2.quat);
}

void ExpectGPSData(const double lat, const double lon, const double alt,
                   const GpsData& gps) {
  EXPECT_DOUBLE_EQ(lat, gps.lat);
  EXPECT_DOUBLE_EQ(lon, gps.lon);
  EXPECT_DOUBLE_EQ(alt, gps.alt);
}
#endif  // CORE_TEST_INCLUDE_UTEST_UTEST_DATA_HPP_
