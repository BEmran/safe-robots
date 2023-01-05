// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <array>
#include <string_view>

#include "core/utils/math.hpp"
#include "my_math/quaternion.hpp"

using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::Quat;
using core::utils::Vec3;

constexpr float EPS = 0.00001f;

::testing::AssertionResult operator&&(const ::testing::AssertionResult& res1,
                                      const ::testing::AssertionResult& res2) {
  if (res1) {
    return res2;
  } else {
    return res1;
  }
}

::testing::AssertionResult ExpectEq(const float expect, const float actual,
                                    std::string_view msg = "") {
  if (std::abs(expect - actual) > EPS) {
    return ::testing::AssertionFailure()
           << "Failure test " << msg << " -> "
           << "expect (" << expect << ") and actual (" << actual << ")";
  }
  return ::testing::AssertionSuccess();
}

// [[nodiscard]] ::testing::AssertionResult ExpectEqVec(const Vec3& expect,
//                                                      const Vec3& actual) {
//   auto result = ::testing::AssertionSuccess();
//   for (size_t idx = 0; idx < expect.size(); idx++) {
//     result = result && ExpectEq(expect[idx], actual[idx],
//                                 "idx [" + std::to_string(idx) + "]");
//   }
//   return result;
// }

[[nodiscard]] ::testing::AssertionResult
ExpectEqQuat(const Quaternion& expect, const Quaternion& actual) {
  return ExpectEq(expect.W(), actual.W(), "W component") &&
         ExpectEq(expect.X(), actual.X(), "X component") &&
         ExpectEq(expect.Y(), actual.Y(), "Y component") &&
         ExpectEq(expect.Z(), actual.Z(), "Z component");
}

float Norm(const float w, const float x, const float y, const float z) {
  return std::sqrt(w * w + x * x + y * y + z * z);
}

float Norm(const Quaternion& quat) {
  return Norm(quat.W(), quat.X(), quat.Y(), quat.Z());
}

TEST(Quaternion, DefaultConstructConstructIdentityQuaternion) {
  Quaternion q;
  EXPECT_TRUE(ExpectEqQuat({1.f, 0.f, 0.f, 0.f}, q));
}

TEST(Quaternion, ConstructWithScalarAndVector) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  Quaternion q(scalar, vec);
  EXPECT_TRUE(ExpectEqQuat({scalar, vec.x(), vec.y(), vec.z()}, q));
}

TEST(Quaternion, ConstructWithWXYZ) {
  const float w{+0.2f};
  const float x{-0.4f};
  const float y{+0.6f};
  const float z{-0.8f};
  Quaternion q(w, x, y, z);
  EXPECT_TRUE(ExpectEqQuat({w, x, y, z}, q));
}

TEST(Quaternion, NormForIdentity) {
  EXPECT_TRUE(ExpectEq(1.f, Quaternion().Norm()));
}

TEST(Quaternion, NormTest) {
  const float w{+0.2f};
  const float x{-0.4f};
  const float y{+0.6f};
  const float z{-0.8f};
  Quaternion q(w, x, y, z);
  EXPECT_TRUE(ExpectEq(Norm(w, x, y, z), q.Norm()));
}

TEST(Quaternion, Normalize) {
  const float w{+0.2f};
  const float x{-1.4f};
  const float y{+1.6f};
  const float z{-1.8f};
  Quaternion actual_quat(w, x, y, z);
  actual_quat.Normalize();

  const float expect_norm = Norm(w, x, y, z);
  const Quaternion expect_quat(w / expect_norm, x / expect_norm,
                               y / expect_norm, z / expect_norm);
  EXPECT_TRUE(ExpectEqQuat(expect_quat, actual_quat));
}

TEST(Quaternion, NormalizeZeroQuaternion) {
  Quaternion actual_quat(0, 0, 0, 0);
  actual_quat.Normalize();
  EXPECT_TRUE(ExpectEqQuat({1.f, 0.f, 0.f, 0.f}, actual_quat));
}

TEST(Quaternion, Normalized) {
  const float w{+0.2f};
  const float x{-1.4f};
  const float y{+1.6f};
  const float z{-1.8f};
  Quaternion actual_quat(w, x, y, z);

  const float expect_norm = Norm(w, x, y, z);
  const Quaternion expect_quat(w / expect_norm, x / expect_norm,
                               y / expect_norm, z / expect_norm);
  EXPECT_TRUE(ExpectEqQuat(expect_quat, actual_quat.Normalized()));
  EXPECT_TRUE(ExpectEqQuat({w, x, y, z}, actual_quat));
}

TEST(Quaternion, NormalizedZeroQuaternion) {
  Quaternion actual_quat(0.f, 0.f, 0.f, 0.f);
  EXPECT_TRUE(ExpectEqQuat({1.f, 0.f, 0.f, 0.f}, actual_quat.Normalized()));
  EXPECT_TRUE(ExpectEqQuat({0.f, 0.f, 0.f, 0.f}, actual_quat));
}
