// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "utest/utils_data.hpp"

#include "cmath"
#include "gtest/gtest.h"

constexpr auto kEpsilon = 0.00001F;

::testing::AssertionResult operator&&(::testing::AssertionResult ar1,
                                      ::testing::AssertionResult ar2) {
  return ar1 ? ar1 : ar2;
}

bool IsEqual(float actual, float expect) {
  return std::fabs(actual - expect) < kEpsilon;
}

bool IsEqual(double actual, double expect) {
  return std::fabs(actual - expect) < kEpsilon;
}

::testing::AssertionResult ExpectEq(MATH_TYPE actual, MATH_TYPE expect,
                                    std::string_view extra_msg) {
  if (IsEqual(actual, expect)) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure()
           << extra_msg << " actual: " << actual << " expect: " << expect;
  }
}

::testing::AssertionResult ExpectDoubleDataEq(const DoubleData& d1,
                                              const DoubleData& d2) {
  if (IsEqual(d1.value, d2.value)) {
    return ::testing::AssertionSuccess();
  } else {
    return ::testing::AssertionFailure()
           << "actual: " << d1.value << " expect: " << d2.value;
  }
}

::testing::AssertionResult ExpectVec3Eq(const Vec3& v1, const Vec3& v2) {
  auto e1 = ExpectEq(v1.x(), v2.x(), "x value: ");
  auto e2 = ExpectEq(v1.y(), v2.y(), "y value: ");
  auto e3 = ExpectEq(v1.z(), v2.z(), "z value: ");
  return e1 && e2 && e3;
}

::testing::AssertionResult ExpectVec3DataEq(const Vec3Data& v1,
                                            const Vec3Data& v2) {
  return ExpectVec3Eq(v1.data, v2.data);
}

::testing::AssertionResult ExpectQuatEq(const Quat& q1, const Quat& q2) {
  auto e1 = ExpectVec3Eq(q1.vec(), q2.vec());
  auto e2 = ExpectEq(q1.w(), q2.w());
  return e1 && e2;
}

::testing::AssertionResult ExpectQuatDataEq(const QuatData& q1,
                                            const QuatData& q2) {
  return ExpectQuatEq(q1.data, q2.data);
}

::testing::AssertionResult ExpectImuData(const ImuData& d1, const ImuData& d2) {
  auto e1 = ExpectDoubleDataEq(d1.heading, d2.heading);
  auto e2 = ExpectDoubleDataEq(d1.temp, d2.temp);
  auto e3 = ExpectVec3DataEq(d1.tait_bryan, d2.tait_bryan);
  auto e4 = ExpectVec3DataEq(d1.accel, d2.accel);
  auto e5 = ExpectVec3DataEq(d1.gyro, d2.gyro);
  auto e6 = ExpectVec3DataEq(d1.mag, d2.mag);
  auto e7 = ExpectQuatDataEq(d1.quat, d2.quat);
  return e1 && e2 && e3 && e4 && e5 && e6 && e7;
}

::testing::AssertionResult ExpectGPSData(double lat, double lon, double alt,
                                         const GpsData& gps) {
  auto e1 = ExpectEq(lat, gps.lat, "Latitude value: ");
  auto e2 = ExpectEq(lon, gps.lon, "Longitude value: ");
  auto e3 = ExpectEq(alt, gps.alt, "Altitude value: ");
  return e1 && e2 && e3;
}
