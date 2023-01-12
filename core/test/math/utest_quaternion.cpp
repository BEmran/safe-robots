// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <array>
#include <eigen3/Eigen/Geometry>
#include <string_view>

#include "core/math/math.hpp"
#include "core/math/quaternion.hpp"
#include "core/math/transformation.hpp"
#include "core/utils/logger_macros.hpp"
#include "utest_utils.hpp"

using core::math::Mat3;
using core::math::MATH_TYPE;
using core::math::PI;
using core::math::PI_2;
using core::math::Quat;
using core::math::Quaternion;
using core::math::Vec3;
constexpr float PI_4 = PI / 4.f;

float Mod(const float x, const float val) {
  if (x >= val) {
    return Mod(x - val, val);
  } else if (x <= -val) {
    return Mod(x + val, val);
  } else {
    return x;
  }
}

float Norm(const float w, const float x, const float y, const float z) {
  return std::sqrt(w * w + x * x + y * y + z * z);
}

float Norm(const Quaternion& quat) {
  return Norm(quat.W(), quat.X(), quat.Y(), quat.Z());
}

TEST(Quaternion, DefaultConstructConstructIdentityQuaternion) {
  Quaternion q;
  EXPECT_TRUE(ExpectEqQuaternion({1.f, 0.f, 0.f, 0.f}, q));
}

TEST(Quaternion, ConstructWithScalarAndVector) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  Quaternion q(scalar, vec);
  EXPECT_TRUE(ExpectEqQuaternion({scalar, vec.x(), vec.y(), vec.z()}, q));
}

TEST(Quaternion, ConstructWithWXYZ) {
  const float w{+0.2f};
  const float x{-0.4f};
  const float y{+0.6f};
  const float z{-0.8f};
  Quaternion q(w, x, y, z);
  EXPECT_TRUE(ExpectEqQuaternion({w, x, y, z}, q));
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
  Quaternion actual(w, x, y, z);
  actual.Normalize();

  const float norm = Norm(w, x, y, z);
  const Quaternion expect(w / norm, x / norm, y / norm, z / norm);
  EXPECT_TRUE(ExpectEqQuaternion(expect, actual));
}

TEST(Quaternion, NormalizeZeroQuaternionFixItToIdentity) {
  Quaternion quat(0, 0, 0, 0);
  quat.Normalize();
  EXPECT_TRUE(ExpectEqQuaternion({1.f, 0.f, 0.f, 0.f}, quat));
}

TEST(Quaternion, Normalized) {
  const float w{+0.2f};
  const float x{-1.4f};
  const float y{+1.6f};
  const float z{-1.8f};
  Quaternion actual(w, x, y, z);

  const float norm = Norm(w, x, y, z);
  const Quaternion expect(w / norm, x / norm, y / norm, z / norm);
  EXPECT_TRUE(ExpectEqQuaternion(expect, actual.Normalized()));
  EXPECT_TRUE(ExpectEqQuaternion({w, x, y, z}, actual));
}

TEST(Quaternion, NormalizedZeroQuaternionReturnIdentity) {
  Quaternion actual(0.f, 0.f, 0.f, 0.f);
  EXPECT_TRUE(ExpectEqQuaternion({1.f, 0.f, 0.f, 0.f}, actual.Normalized()));
  EXPECT_TRUE(ExpectEqQuaternion({0.f, 0.f, 0.f, 0.f}, actual));
}

TEST(Quaternion, SetIdentity) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  Quaternion q(scalar, vec);
  EXPECT_TRUE(ExpectEqQuaternion({scalar, vec.x(), vec.y(), vec.z()}, q));
  q.SetIdentity();
  EXPECT_TRUE(ExpectEqQuaternion({1.f, 0.f, 0.f, 0.f}, q));
}

TEST(Quaternion, Conjugate) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);
  const Quaternion expect = q.Conjugate();
  const Quaternion actual(scalar, -vec);
  EXPECT_TRUE(ExpectEqQuaternion(actual, expect));
}

TEST(Quaternion, DoubleConjugateIsTheSameQuaternion) {
  const Quaternion q(0.1f, Vec3{0.2f, 0.3f, 0.4f});
  const Quaternion actual = q.Conjugate().Conjugate();
  EXPECT_TRUE(ExpectEqQuaternion(q, actual));
}

TEST(Quaternion, Dot) {
  const float scalar{0.1f};
  const Vec3 vec1{0.2f, 0.3f, 0.4f};
  const Vec3 vec2{0.6f, 0.7f, 0.8f};
  const Quaternion q1(scalar, vec1);
  const Quaternion q2(scalar, vec2);
  const float actual = q1.Dot(q2);
  const float expect = q1.W() * q2.W() +  //
                       q1.X() * q2.X() +  //
                       q1.Y() * q2.Y() +  //
                       q1.Z() * q2.Z();
  EXPECT_TRUE(ExpectEq(expect, actual));
}

TEST(Quaternion, DotForSameQuaternionIsNormSquared) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);
  const float actual = q.Dot(q);
  const float expect = scalar * scalar + vec.squaredNorm();
  EXPECT_TRUE(ExpectEq(expect, actual));
}

TEST(Quaternion, AngularDistance) {
  const Quaternion q1;
  for (size_t i = -9; i < 9; i++) {
    const float ang = static_cast<float>(i) * PI_4;
    const Quaternion q2{
      Eigen::Quaternionf(Eigen::AngleAxisf(ang, Vec3::UnitY()))};
    const float expect = q1.AngularDistance(q2);
    const float actual = Mod(ang, 2.f * PI);
    EXPECT_TRUE(ExpectEq(actual, expect));
  }
}

TEST(Quaternion, AngularDistanceForSameQuaternionIsZero) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);
  const float expect = q.AngularDistance(q);
  EXPECT_TRUE(ExpectEq(0.f, expect));
}

TEST(Quaternion, AngularDistanceForMirrorQuaternionIsZero) {
  const Quaternion q =
    Eigen::Quaternionf(Eigen::AngleAxisf(PI_4, Vec3::UnitY()));
  const Quaternion mirror(-q.Scalar(), -q.Vec());
  const float expect = q.AngularDistance(mirror);
  EXPECT_TRUE(ExpectEq(0, expect));
}

TEST(Quaternion, AddTwoQuaternion) {
  const float scalar1{0.1f};
  const Vec3 vec1{0.2f, 0.3f, 0.4f};
  const Quaternion q1(scalar1, vec1);
  const float scalar2{0.1f};
  const Vec3 vec2{0.6f, 0.7f, 0.8f};
  const Quaternion q2(scalar2, vec2);

  const Quaternion actual = q1 + q2;
  EXPECT_TRUE(ExpectEq(q1.W() + q2.W(), actual.W()));
  EXPECT_TRUE(ExpectEq(q1.X() + q2.X(), actual.X()));
  EXPECT_TRUE(ExpectEq(q1.Y() + q2.Y(), actual.Y()));
  EXPECT_TRUE(ExpectEq(q1.Z() + q2.Z(), actual.Z()));
}

TEST(Quaternion, SubtractTwoQuaternion) {
  const float scalar1{0.1f};
  const Vec3 vec1{0.2f, 0.3f, 0.4f};
  const Quaternion q1(scalar1, vec1);
  const float scalar2{0.1f};
  const Vec3 vec2{0.6f, 0.7f, 0.8f};
  const Quaternion q2(scalar2, vec2);

  const Quaternion actual = q1 - q2;
  EXPECT_TRUE(ExpectEq(q1.W() - q2.W(), actual.W()));
  EXPECT_TRUE(ExpectEq(q1.X() - q2.X(), actual.X()));
  EXPECT_TRUE(ExpectEq(q1.Y() - q2.Y(), actual.Y()));
  EXPECT_TRUE(ExpectEq(q1.Z() - q2.Z(), actual.Z()));
}

TEST(Quaternion, MultiplyTwoQuaternions) {
  const float scalar1{0.1f};
  const Vec3 vec1{0.2f, 0.3f, 0.4f};
  const Quaternion q1(scalar1, vec1);
  const float scalar2{0.1f};
  const Vec3 vec2{0.6f, 0.7f, 0.8f};
  const Quaternion q2(scalar2, vec2);

  const Quaternion actual = q1 * q2;
  const Quaternion expect(scalar1 * scalar2 - vec1.dot(vec2),
                          scalar1 * vec2 + scalar2 * vec1 + vec1.cross(vec2));
  EXPECT_TRUE(ExpectEqQuaternion(expect, actual));
}

TEST(Quaternion, MultiplySameQuaternion) {
  const float scalar{0.1f};
  const Vec3 vec{0.2f, 0.3f, 0.4f};
  const Quaternion q(scalar, vec);

  const Quaternion expect = q * q;
  const Quaternion actual(scalar * scalar - vec.squaredNorm(),
                          2 * scalar * vec);
  EXPECT_TRUE(ExpectEqQuaternion(expect, actual));
}

TEST(Quaternion, RotateAVector) {
  const Quaternion q(0.1f, Vec3{0.2f, 0.3f, 0.4f});

  const Vec3 Vector{0.5f, 0.6f, 0.7f};
  const Quaternion actual = q * Vector;
  const Quaternion expect = q * Quaternion(0, Vector) * q.Conjugate();
  EXPECT_TRUE(ExpectEqQuaternion(expect, actual));
}

TEST(Quaternion, MultiplyByConstantLHS) {
  const Quaternion q(0.1f, Vec3{0.2f, 0.3f, 0.4f});
  const float constant{3.f};

  const Quaternion actual = q * constant;
  EXPECT_TRUE(ExpectEq(q.W() * constant, actual.W()));
  EXPECT_TRUE(ExpectEq(q.X() * constant, actual.X()));
  EXPECT_TRUE(ExpectEq(q.Y() * constant, actual.Y()));
  EXPECT_TRUE(ExpectEq(q.Z() * constant, actual.Z()));
}

TEST(Quaternion, MultiplyByConstantRHS) {
  const Quaternion q(0.1f, Vec3{0.2f, 0.3f, 0.4f});
  const float constant{3.f};

  const Quaternion actual = constant * q;
  EXPECT_TRUE(ExpectEq(q.W() * constant, actual.W()));
  EXPECT_TRUE(ExpectEq(q.X() * constant, actual.X()));
  EXPECT_TRUE(ExpectEq(q.Y() * constant, actual.Y()));
  EXPECT_TRUE(ExpectEq(q.Z() * constant, actual.Z()));
}

TEST(Quaternion, DivideByConstant) {
  const Quaternion q(0.1f, Vec3{0.2f, 0.3f, 0.4f});
  const float constant{3.f};

  const Quaternion actual = q / constant;
  EXPECT_TRUE(ExpectEq(q.W() / constant, actual.W()));
  EXPECT_TRUE(ExpectEq(q.X() / constant, actual.X()));
  EXPECT_TRUE(ExpectEq(q.Y() / constant, actual.Y()));
  EXPECT_TRUE(ExpectEq(q.Z() / constant, actual.Z()));
}

TEST(LinearInterpolation, Linear) {
  SYS_LOG_INFO("0: ") << Quaternion().AngularDistance({0.0f, 0.2f, 0.3f, 0.4f});
  SYS_LOG_INFO("1: ") << Quaternion().AngularDistance({0.1f, 0.2f, 0.3f, 0.4f});
  SYS_LOG_INFO("2: ") << Quaternion().AngularDistance({0.2f, 0.1f, 0.3f, 0.4f});
  SYS_LOG_INFO("3: ") << Quaternion().AngularDistance({0.3f, 0.1f, 0.2f, 0.4f});
  SYS_LOG_INFO("4: ") << Quaternion().AngularDistance({0.4f, 0.1f, 0.2f, 0.3f});
  SYS_LOG_INFO("5: ") << Quaternion().AngularDistance({0.5f, 0.1f, 0.2f, 0.3f});
  SYS_LOG_INFO("6: ") << Quaternion().AngularDistance({0.6f, 0.1f, 0.2f, 0.3f});

  SYS_LOG_INFO("-0: ") << Quaternion().AngularDistance(
    {-0.0f, 0.2f, 0.3f, 0.4f});
  SYS_LOG_INFO("-1: ") << Quaternion().AngularDistance(
    {-0.1f, 0.2f, 0.3f, 0.4f});
  SYS_LOG_INFO("-2: ") << Quaternion().AngularDistance(
    {-0.2f, 0.1f, 0.3f, 0.4f});
  SYS_LOG_INFO("-3: ") << Quaternion().AngularDistance(
    {-0.3f, 0.1f, 0.2f, 0.4f});
  SYS_LOG_INFO("-4: ") << Quaternion().AngularDistance(
    {-0.4f, 0.1f, 0.2f, 0.3f});
  SYS_LOG_INFO("-5: ") << Quaternion().AngularDistance(
    {-0.5f, 0.1f, 0.2f, 0.3f});
  SYS_LOG_INFO("-6: ") << Quaternion().AngularDistance(
    {-0.6f, 0.1f, 0.2f, 0.3f});
}

/*****************************************************************************/
class QuaternionVsEigen : public testing::Test {
 public:
  void SetUp() override {
    e1 = Eigen::Quaternionf(scalar1, vec1.x(), vec1.y(), vec1.z());
    e2 = Eigen::Quaternionf(scalar2, vec2.x(), vec2.y(), vec2.z());
    q1 = Quaternion(e1);
    q2 = Quaternion(e2);
  }

  float scalar1{0.1f};
  float scalar2{0.2f};
  Vec3 vec1{0.2f, 0.3f, 0.4f};
  Vec3 vec2{0.6f, 0.7f, 0.8f};
  Quaternion q1;
  Quaternion q2;
  Eigen::Quaternionf e1;
  Eigen::Quaternionf e2;
};

TEST_F(QuaternionVsEigen, Norm) {
  const float expect = q1.Norm();
  const float actual = e1.norm();
  EXPECT_TRUE(ExpectEq(actual, expect));
}

TEST_F(QuaternionVsEigen, Normalize) {
  q1.Normalize();
  e1.normalize();
  EXPECT_TRUE(ExpectEqQuaternion(e1, q1));
}

TEST_F(QuaternionVsEigen, Normalized) {
  const Quaternion expect = q1.Normalized();
  const Eigen::Quaternionf actual = e1.normalized();
  EXPECT_TRUE(ExpectEqQuaternion(actual, expect));
}

TEST_F(QuaternionVsEigen, Dot) {
  const float expect = q1.Dot(q2);
  const float actual = e1.dot(e2);
  EXPECT_TRUE(ExpectEq(actual, expect));
}

TEST_F(QuaternionVsEigen, AngularDistance) {
  const float expect = q1.AngularDistance(q2);
  const float actual = e1.angularDistance(e2);
  EXPECT_TRUE(ExpectEq(actual, expect));
}

TEST_F(QuaternionVsEigen, Conjugate) {
  const Quaternion expect = q1.Conjugate();
  const Eigen::Quaternionf actual = e1.conjugate();
  EXPECT_TRUE(ExpectEqQuaternion(actual, expect));
}

TEST_F(QuaternionVsEigen, MultiplyTwoQuaternions) {
  const Quaternion expect = q1 * q2;
  const Eigen::Quaternionf actual = e1 * e2;
  EXPECT_TRUE(ExpectEqQuaternion(actual, expect));
}

TEST_F(QuaternionVsEigen, Angle) {
  for (size_t i = -9; i < 9; i++) {
    const float ang = static_cast<float>(i) * PI_4;
    const Vec3 vec = Vec3::UnitY();
    const Eigen::Quaternionf expect(Eigen::AngleAxisf(ang, vec));
    const Quaternion actual{expect};

    const std::string msg = "angle (" + std::to_string(ang) + ")";
    EXPECT_TRUE(ExpectEq(Mod(ang, 2 * PI), actual.Angle(), msg));
    EXPECT_TRUE(ExpectEq(1.f, core::math::Sign(actual.Y()), msg));
  }
}