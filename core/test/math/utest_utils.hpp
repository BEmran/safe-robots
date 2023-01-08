// Copyright (C) 2022 Bara Emran - All Rights Reserved
#ifndef CORE_MATH_UTILS_HPP_
#define CORE_MATH_UTILS_HPP_

#include <gtest/gtest.h>

#include "core/utils/math.hpp"
#include "my_math/dcm.hpp"

using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::Quat;
using core::utils::Vec3;

[[nodiscard]] ::testing::AssertionResult
operator&&(const ::testing::AssertionResult& res1,
           const ::testing::AssertionResult& res2);

[[nodiscard]] ::testing::AssertionResult ExpectEq(const MATH_TYPE expect,
                                                  const MATH_TYPE actual,
                                                  std::string_view msg = "");

[[nodiscard]] ::testing::AssertionResult ExpectEqMat3(const Mat3& expect,
                                                      const Mat3& actual);

[[nodiscard]] ::testing::AssertionResult ExpectEqMat3(const my::DCM& expect,
                                                      const my::DCM& actual);

[[nodiscard]] ::testing::AssertionResult ExpectEqVec3(const Vec3& expect,
                                                      const Vec3& actual);

[[nodiscard]] ::testing::AssertionResult
ExpectEqRPY(const core::utils::RPY& expect, const core::utils::RPY& actual);

[[nodiscard]] ::testing::AssertionResult ExpectEqQuat(const Quat& expect,
                                                      const Quat& actual);

#endif  // CORE_MATH_UTILS_HPP_