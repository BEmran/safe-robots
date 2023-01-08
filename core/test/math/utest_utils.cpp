// Copyright (C) 2022 Bara Emran - All Rights Reserved
#include "utest_utils.hpp"

#include <eigen3/Eigen/Core>

constexpr float EPS = 0.00001f;

[[nodiscard]] ::testing::AssertionResult
operator&&(const ::testing::AssertionResult& res1,
           const ::testing::AssertionResult& res2) {
  if (res1) {
    return res2;
  } else {
    return res1;
  }
}

[[nodiscard]] ::testing::AssertionResult ExpectEq(const MATH_TYPE expect,
                                                  const MATH_TYPE actual,
                                                  std::string_view msg) {
  if (std::abs(expect - actual) > EPS) {
    return ::testing::AssertionFailure()
           << "Failure test " << msg << " -> "
           << "expect (" << expect << ") and actual (" << actual << ")";
  }
  return ::testing::AssertionSuccess();
}

[[nodiscard]] ::testing::AssertionResult ExpectEqMat3(const Mat3& expect,
                                                      const Mat3& actual) {
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  for (Eigen::Index i = 0; i < 9; i++) {
    std::stringstream msg;
    msg << "idx(" << i << ") at Mat[" << i / 3 << "," << i % 3l << "]";
    result = result && ExpectEq(expect(i), actual(i), msg.str());
  }
  return result;
}

[[nodiscard]] ::testing::AssertionResult ExpectEqMat3(const my::DCM& expect,
                                                      const my::DCM& actual) {
  return ExpectEqMat3(expect.Matrix(), actual.Matrix());
}

[[nodiscard]] ::testing::AssertionResult ExpectEqVec3(const Vec3& expect,
                                                      const Vec3& actual) {
  for (Eigen::Index i = 0; i < expect.size(); i++) {
    if (std::abs(expect(i) - actual(i)) > EPS) {
      return ExpectEq(expect(i), actual(i), "idx(" + std::to_string(i) + ")");
    }
  }
  return ::testing::AssertionSuccess();
}

[[nodiscard]] ::testing::AssertionResult
ExpectEqRPY(const core::utils::RPY& expect, const core::utils::RPY& actual) {
  return ExpectEq(expect.roll, actual.roll, "roll Component") &&
         ExpectEq(expect.pitch, actual.pitch, "pitch Component") &&
         ExpectEq(expect.yaw, actual.yaw, "yaw Component");
}

[[nodiscard]] ::testing::AssertionResult
ExpectEqQuaternion(const my::Quaternion& expect, const my::Quaternion& actual) {
  if (std::abs(expect.W() - actual.W()) < EPS) {
    return ExpectEqVec3(expect.Vec(), actual.Vec());
  } else if (std::abs(expect.W()) - std::abs(actual.W()) < EPS) {
    return ExpectEqVec3(expect.Vec(), Vec3(-1.f * actual.Vec()))  //
           << " needed to flip real part";
  } else {
    return ::testing::AssertionFailure()
           << "error real part -> "
           << "expect (" << expect.W() << ") and actual (" << actual.W() << ")";
  }
}

Mat3 RandomRotationMatrix() {
  return core::utils::Quat::UnitRandom().toRotationMatrix();
}