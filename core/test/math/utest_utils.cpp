// Copyright (C) 2022 Bara Emran - All Rights Reserved
#include "utest_utils.hpp"

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
  for (Eigen::Index i = 0; i < expect.size(); i++) {
    std::stringstream msg;
    const long row = i / expect.cols();
    const long col = i % expect.cols();
    msg << "idx(" << i << ") at Mat[" << row << "," << col << "]";
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

[[nodiscard]] ::testing::AssertionResult ExpectEqQuat(const Quat& expect,
                                                      const Quat& actual) {
  if (std::abs(expect.w() - actual.w()) < EPS) {
    return ExpectEqVec3(Vec3(expect.vec()), Vec3(actual.vec()));
  } else if (std::abs(expect.w()) - std::abs(actual.w()) < EPS) {
    return ExpectEqVec3(Vec3(expect.vec()), Vec3(-1.f * actual.vec()))  //
           << " needed to flip angle";
  } else {
    return ::testing::AssertionFailure()
           << "error angle -> "
           << "expect (" << expect.w() << ") and actual (" << actual.w() << ")";
  }
}