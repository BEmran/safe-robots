// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef UTEST_UTEST_DATA_HPP_
#define UTEST_UTEST_DATA_HPP_

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

bool Equal(MATH_TYPE actual, MATH_TYPE expect);

void ExpectEq(MATH_TYPE actual, MATH_TYPE expect);

void ExpectDoubleDataEq(const DoubleData& d1, const DoubleData& d2);

void ExpectVec3Eq(const Vec3& v1, const Vec3& v2);

void ExpectVec3DataEq(const Vec3Data& v1, const Vec3Data& v2);

void ExpectQuatEq(const Quat& q1, const Quat& q2);

void ExpectQuatDataEq(const QuatData& q1, const QuatData& q2);

void ExpectImuData(const ImuData& d1, const ImuData& d2);

void ExpectGPSData(double lat, double lon, double alt, const GpsData& gps);
#endif  // UTEST_UTEST_DATA_HPP_
