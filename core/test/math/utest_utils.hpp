// Copyright (C) 2022 Bara Emran - All Rights Reserved
#ifndef CORE_MATH_UTILS_HPP_
#define CORE_MATH_UTILS_HPP_

#include <gtest/gtest.h>

#include "core/math/dcm.hpp"
#include "core/math/math.hpp"
#include "core/math/quaternion.hpp"

using core::math::Mat3;
using core::math::MATH_TYPE;
using core::math::Quat;
using core::math::Vec3;

[[nodiscard]] ::testing::AssertionResult
operator&&(const ::testing::AssertionResult& res1,
           const ::testing::AssertionResult& res2);

[[nodiscard]] ::testing::AssertionResult ExpectEq(const MATH_TYPE expect,
                                                  const MATH_TYPE actual,
                                                  std::string_view msg = "");

[[nodiscard]] ::testing::AssertionResult ExpectEqMat3(const Mat3& expect,
                                                      const Mat3& actual);

[[nodiscard]] ::testing::AssertionResult
ExpectEqMat3(const core::math::DCM& expect, const core::math::DCM& actual);

[[nodiscard]] ::testing::AssertionResult ExpectEqVec3(const Vec3& expect,
                                                      const Vec3& actual);

[[nodiscard]] ::testing::AssertionResult
ExpectEqRPY(const core::math::RPY& expect, const core::math::RPY& actual);

[[nodiscard]] ::testing::AssertionResult ExpectEqQuaternion(
  const core::math::Quaternion& expect, const core::math::Quaternion& actual);

Mat3 RandomRotationMatrix();
#endif  // CORE_MATH_UTILS_HPP_