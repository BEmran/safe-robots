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

TEST(Quaternion, SetIdentity) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  Quaternion q(scalar, vec);
  EXPECT_TRUE(ExpectEqQuat({scalar, vec.x(), vec.y(), vec.z()}, q));
  q.SetIdentity();
  EXPECT_TRUE(ExpectEqQuat({1.f, 0.f, 0.f, 0.f}, q));
}

TEST(Quaternion, GetComponents) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);
  EXPECT_TRUE(ExpectEq(scalar, q.W()));
  EXPECT_TRUE(ExpectEq(vec.x(), q.X()));
  EXPECT_TRUE(ExpectEq(vec.y(), q.Y()));
  EXPECT_TRUE(ExpectEq(vec.z(), q.Z()));
}

TEST(Quaternion, ModifyComponent) {
  Quaternion q;
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  q.W() = scalar;
  q.X() = vec.x();
  q.Y() = vec.y();
  q.Z() = vec.z();

  EXPECT_TRUE(ExpectEq(scalar, q.W()));
  EXPECT_TRUE(ExpectEq(vec.x(), q.X()));
  EXPECT_TRUE(ExpectEq(vec.y(), q.Y()));
  EXPECT_TRUE(ExpectEq(vec.z(), q.Z()));
}

TEST(Quaternion, Conjugate) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);
  const Quaternion expect = q.Conjugate();
  const Quaternion actual(scalar, -vec);
  EXPECT_TRUE(ExpectEqQuat(actual, expect));
}

TEST(Quaternion, DoubleConjugateIsTheSameQuaternion) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);
  const Quaternion expect = q.Conjugate().Conjugate();
  EXPECT_TRUE(ExpectEqQuat(q, expect));
}

TEST(Quaternion, Dot) {
  const float scalar{0.1f};
  const Vec3 vec1{0.2f, 0.3f, 0.4f};
  const Vec3 vec2{0.6f, 0.7f, 0.8f};
  const Quaternion q1(scalar, vec1);
  const Quaternion q2(scalar, vec2);
  const float expect = q1.Dot(q2);
  const float actual = q1.X() * q2.X() +  //
                       q1.Y() * q2.Y() +  //
                       q1.Z() * q2.Z();
  EXPECT_TRUE(ExpectEq(actual, expect));
}

TEST(Quaternion, DotForSameQuaternionIsNormSquared) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);
  const float expect = q.Dot(q);
  EXPECT_TRUE(ExpectEq(vec.squaredNorm(), expect));
}

TEST(Quaternion, Cross) {
  const float scalar{0.1f};
  const Vec3 vec1{0.2f, 0.3f, 0.4f};
  const Vec3 vec2{0.6f, 0.7f, 0.8f};
  const Quaternion q1(scalar, vec1);
  const Quaternion q2(scalar, vec2);
  const Vec3 expect = q1.Cross(q2);
  const float actual_x = q1.Y() * q2.Z() - q1.Z() * q2.Y();  //
  const float actual_y = q1.Z() * q2.X() - q1.X() * q2.Z();  //
  const float actual_z = q1.X() * q2.Y() - q1.Y() * q2.X();
  EXPECT_TRUE(ExpectEq(actual_x, expect.x()));
  EXPECT_TRUE(ExpectEq(actual_y, expect.y()));
  EXPECT_TRUE(ExpectEq(actual_z, expect.z()));
}

TEST(Quaternion, CrossForSameQuaternionIsZero) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);
  const Vec3 expect = q.Cross(q);
  EXPECT_TRUE(ExpectEq(0.f, expect.x()));
  EXPECT_TRUE(ExpectEq(0.f, expect.y()));
  EXPECT_TRUE(ExpectEq(0.f, expect.z()));
}

TEST(Quaternion, CrossForPerpendicularVector) {
  const float scalar{0.1f};
  const Vec3 vec_x{1.f, 0.f, 0.f};
  const Vec3 vec_y{0.f, 1.f, 0.f};
  const Vec3 vec_z{0.f, 0.f, 1.f};
  const Quaternion qx(scalar, vec_x);
  const Quaternion qy(scalar, vec_y);
  const Vec3 expect = qx.Cross(qy);
  EXPECT_TRUE(ExpectEq(vec_z.x(), expect.x()));
  EXPECT_TRUE(ExpectEq(vec_z.y(), expect.y()));
  EXPECT_TRUE(ExpectEq(vec_z.z(), expect.z()));
}

TEST(Quaternion, AngularDistance) {
  const float scalar{0.1f};
  const Vec3 vec1{0.2f, 0.3f, 0.4f};
  const Vec3 vec2{0.6f, 0.7f, 0.8f};
  const Quaternion q1(scalar, vec1);
  const Quaternion q2(scalar, vec2);
  const float expect = q1.AngularDistance(q2);
  const float actual = std::acos(vec1.normalized().dot(vec2.normalized()));
  EXPECT_TRUE(ExpectEq(actual, expect));
}

TEST(Quaternion, AngularDistanceForSameQuaternionIsZero) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);
  const float expect = q.AngularDistance(q);
  EXPECT_TRUE(ExpectEq(0.f, expect));
}

TEST(Quaternion, AddTwoQuaternion) {
  const float scalar1{0.1f};
  const Vec3 vec1{0.2f, 0.3f, 0.4f};
  const Quaternion q1(scalar1, vec1);
  const float scalar2{0.1f};
  const Vec3 vec2{0.6f, 0.7f, 0.8f};
  const Quaternion q2(scalar2, vec2);

  const Quaternion expect = q1 + q2;
  const Quaternion actual(scalar1 + scalar2, vec1 + vec2);
  EXPECT_TRUE(ExpectEqQuat(actual, expect));
}

TEST(Quaternion, SubtractTwoQuaternion) {
  const float scalar1{0.1f};
  const Vec3 vec1{0.2f, 0.3f, 0.4f};
  const Quaternion q1(scalar1, vec1);
  const float scalar2{0.1f};
  const Vec3 vec2{0.6f, 0.7f, 0.8f};
  const Quaternion q2(scalar2, vec2);

  const Quaternion expect = q1 - q2;
  const Quaternion actual(scalar1 - scalar2, vec1 - vec2);
  EXPECT_TRUE(ExpectEqQuat(actual, expect));
}

TEST(Quaternion, MultiplyTwoQuaternions) {
  const float scalar1{0.1f};
  const Vec3 vec1{0.2f, 0.3f, 0.4f};
  const Quaternion q1(scalar1, vec1);
  const float scalar2{0.1f};
  const Vec3 vec2{0.6f, 0.7f, 0.8f};
  const Quaternion q2(scalar2, vec2);

  const Quaternion expect = q1 * q2;
  const Quaternion actual(scalar1 * scalar2 - vec1.dot(vec2),
                          scalar1 * vec2 + scalar2 * vec1 + vec2.cross(vec1));
  EXPECT_TRUE(ExpectEqQuat(actual, expect));
}

TEST(Quaternion, MultiplySameQuaternion) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);

  const Quaternion expect = q * q;
  const Quaternion actual(scalar * scalar - vec.squaredNorm(),
                          2 * scalar * vec);
  EXPECT_TRUE(ExpectEqQuat(actual, expect));
}

TEST(Quaternion, RotateAVector) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Vec3 Vector{0.5f, 0.6f, 0.7f};
  const Quaternion q(scalar, vec);

  const Quaternion expect = q * Vector;
  const Quaternion actual = q * Quaternion(0, Vector) * q.Conjugate();
  EXPECT_TRUE(ExpectEqQuat(actual, expect));
}

TEST(Quaternion, MultiplyByConstant) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);
  const float constant{3.f};

  const Quaternion expect = q * constant;
  const Quaternion actual(scalar * constant, vec * constant);
  EXPECT_TRUE(ExpectEqQuat(actual, expect));
}

TEST(Quaternion, DivideByConstant) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);
  const float constant{3.f};

  const Quaternion expect = q / constant;
  const Quaternion actual(scalar / constant, vec / constant);
  EXPECT_TRUE(ExpectEqQuat(actual, expect));
}