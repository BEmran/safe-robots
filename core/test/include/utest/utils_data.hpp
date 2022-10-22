// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef UTEST_UTEST_DATA_HPP_
#define UTEST_UTEST_DATA_HPP_

#include <gtest/gtest.h>

#include <string_view>

#include "core/utils/data.hpp"

using core::utils::DoubleData;
using core::utils::GpsData;
using core::utils::ImuData;
using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::Quat;
using core::utils::QuatData;
using core::utils::Vec3;
using core::utils::Vec3Data;

bool IsEqual(float actual, float expect);

bool IsEqual(double actual, double expect);

::testing::AssertionResult operator&&(::testing::AssertionResult ar1,
                                      ::testing::AssertionResult ar2);

::testing::AssertionResult ExpectEq(MATH_TYPE actual, MATH_TYPE expect,
                                    std::string_view extra_msg = "");

::testing::AssertionResult ExpectDoubleDataEq(const DoubleData& d1,
                                              const DoubleData& d2);

::testing::AssertionResult ExpectVec3Eq(const Vec3& v1, const Vec3& v2);

::testing::AssertionResult ExpectVec3DataEq(const Vec3Data& v1,
                                            const Vec3Data& v2);

::testing::AssertionResult ExpectQuatEq(const Quat& q1, const Quat& q2);

::testing::AssertionResult ExpectQuatDataEq(const QuatData& q1,
                                            const QuatData& q2);

::testing::AssertionResult ExpectImuData(const ImuData& d1, const ImuData& d2);

::testing::AssertionResult ExpectGPSData(double lat, double lon, double alt,
                                         const GpsData& gps);

#endif  // UTEST_UTEST_DATA_HPP_
