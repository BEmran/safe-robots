// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef UTEST_UTEST_DATA_HPP_
#define UTEST_UTEST_DATA_HPP_

#include <gtest/gtest.h>

#include <string_view>

#include "core/utils/data.hpp"
#include "utest/utils.hpp"

using core::utils::DoubleData;
using core::utils::GpsData;
using core::utils::ImuData;
using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::Quat;
using core::utils::QuatData;
using core::utils::Vec3;
using core::utils::Vec3Data;

/**
 * @brief Check if values are equal
 *
 * @param expect expected DoubleData object
 * @param actual actual DoubleData object
 * @return ::testing::AssertionResult assentation result
 */
::testing::AssertionResult ExpectDoubleDataEq(const DoubleData& expect,
                                              const DoubleData& actual);

/**
 * @brief Check if values are equal
 *
 * @param expect expected Vec3
 * @param actual actual Vec3
 * @return ::testing::AssertionResult assentation result
 */
::testing::AssertionResult ExpectVec3Eq(const Vec3& expect, const Vec3& actual);

/**
 * @brief Check if values are equal
 *
 * @param expect expected Vec3Data
 * @param actual actual Vec3Data
 * @return ::testing::AssertionResult assentation result
 */
::testing::AssertionResult ExpectVec3DataEq(const Vec3Data& expect,
                                            const Vec3Data& actual);

/**
 * @brief Check if values are equal
 *
 * @param expect expected Quat
 * @param actual actual Quat
 * @return ::testing::AssertionResult assentation result
 */
::testing::AssertionResult ExpectQuatEq(const Quat& expect, const Quat& actual);

/**
 * @brief Check if values are equal
 *
 * @param expect expected QuatData
 * @param actual actual QuatData
 * @return ::testing::AssertionResult assentation result
 */
::testing::AssertionResult ExpectQuatDataEq(const QuatData& expect,
                                            const QuatData& actual);

/**
 * @brief Check if values are equal
 *
 * @param expect expected ImuData
 * @param actual actual ImuData
 * @return ::testing::AssertionResult assentation result
 */
::testing::AssertionResult ExpectImuData(const ImuData& expect,
                                         const ImuData& actual);

/**
 * @brief Check if gps data are equal
 *
 * @param expect expected GpsData
 * @param actual actual GpsData
 * @return ::testing::AssertionResult assentation result
 */
::testing::AssertionResult ExpectGPSData(const GpsData& expect,
                                         const GpsData& actual);

#endif  // UTEST_UTEST_DATA_HPP_
