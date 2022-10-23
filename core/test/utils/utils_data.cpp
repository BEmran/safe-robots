// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "utest/utils_data.hpp"

#include "gtest/gtest.h"
#include "utest/utils.hpp"

::testing::AssertionResult ExpectDoubleDataEq(const DoubleData& expect,
                                              const DoubleData& actual) {
  return (ExpectEq(expect.value, actual.value));
}

::testing::AssertionResult ExpectVec3Eq(const Vec3& expect,
                                        const Vec3& actual) {
  auto e1 = ExpectEq(expect.x(), actual.x(), "x-value:");
  auto e2 = ExpectEq(expect.y(), actual.y(), "y-value:");
  auto e3 = ExpectEq(expect.z(), actual.z(), "z-value:");
  return e1 && e2 && e3;
}

::testing::AssertionResult ExpectVec3DataEq(const Vec3Data& expect,
                                            const Vec3Data& actual) {
  return ExpectVec3Eq(expect.data, actual.data);
}

::testing::AssertionResult ExpectQuatEq(const Quat& expect,
                                        const Quat& actual) {
  auto e1 = ExpectVec3Eq(expect.vec(), actual.vec());
  auto e2 = ExpectEq(expect.w(), actual.w());
  return e1 && e2;
}

::testing::AssertionResult ExpectQuatDataEq(const QuatData& expect,
                                            const QuatData& actual) {
  return ExpectQuatEq(expect.data, actual.data);
}

::testing::AssertionResult ExpectImuData(const ImuData& expect,
                                         const ImuData& actual) {
  auto e1 = ExpectDoubleDataEq(expect.heading, actual.heading);
  auto e2 = ExpectDoubleDataEq(expect.temp, actual.temp);
  auto e3 = ExpectVec3DataEq(expect.tait_bryan, actual.tait_bryan);
  auto e4 = ExpectVec3DataEq(expect.accel, actual.accel);
  auto e5 = ExpectVec3DataEq(expect.gyro, actual.gyro);
  auto e6 = ExpectVec3DataEq(expect.mag, actual.mag);
  auto e7 = ExpectQuatDataEq(expect.quat, actual.quat);
  return e1 && e2 && e3 && e4 && e5 && e6 && e7;
}

::testing::AssertionResult ExpectGPSData(const GpsData& expect,
                                         const GpsData& actual) {
  auto e1 = ExpectEq(expect.lat, actual.lat, "Latitude value:");
  auto e2 = ExpectEq(expect.lon, actual.lon, "Longitude value:");
  auto e3 = ExpectEq(expect.alt, actual.alt, "Altitude value:");
  return e1 && e2 && e3;
}
