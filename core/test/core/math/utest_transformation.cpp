// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/math/dcm.hpp"
#include "core/math/math.hpp"
#include "core/math/transformation.hpp"
#include "math_utils.hpp"

using core::math::AxisAngleToDCM;
using core::math::AxisAngleToQuaternion;
using core::math::DCM;
using core::math::DCMToAxisAngle;
using core::math::DCMToEuler;
using core::math::DCMToQuaternion;
using core::math::EulerOrder;
using core::math::EulerToDCM;
using core::math::Mat3;
using core::math::MATH_TYPE;
using core::math::Quat;
using core::math::Quaternion;
using core::math::QuaternionMethod;
using core::math::QuaternionToAxisAngle;
using core::math::QuaternionToDCM;
using core::math::RPY;
using core::math::Sign;
using core::math::Skew;
using core::math::Vec3;

TEST(Sign, Negative) {
  EXPECT_TRUE(Sign(-5.F) < 0);
}

TEST(Sign, Positive) {
  EXPECT_TRUE(Sign(5.F) > 0);
}

TEST(Sign, Zero) {
  EXPECT_TRUE(Sign(0) == 0);
}

TEST(Skew, SkewSymmetric) {
  const Vec3 vec(0.2F, -0.4F, 0.8F);
  Mat3 s = Skew(vec);
  EXPECT_TRUE(ExpectEqMat3(Mat3::Zero(), s + s.transpose()));
}

TEST(Skew, SkewSymmetric2) {
  const Vec3 vec(0.5F, -0.2F, 0.1F);
  Mat3 s = Skew(vec);
  EXPECT_TRUE(ExpectEqMat3(-s, s.transpose()));
}

TEST(Skew, DiagonalAreZeros) {
  const Vec3 vec(0.5F, -0.2F, 0.1F);
  Mat3 s = Skew(vec);
  EXPECT_TRUE(s.diagonal().isZero());
}

TEST(Skew, CheckVectorValues) {
  const Vec3 vec(0.5F, -0.2F, 0.1F);
  Mat3 s = Skew(vec);
  EXPECT_FLOAT_EQ(vec.x(), s(2, 1));
  EXPECT_FLOAT_EQ(vec.y(), s(0, 2));
  EXPECT_FLOAT_EQ(vec.z(), s(1, 0));
}

TEST(AxisAngleToRotation, FirstQuarterAngleRandomVector) {
  Eigen::AngleAxisf aa(0.2F, Vec3::Random().normalized());
  EXPECT_TRUE(
    ExpectEqMat3(aa.toRotationMatrix(), AxisAngleToDCM(aa.angle(), aa.axis())));
}

TEST(AxisAngleToRotation, SecondQuarterAngleRandomVector) {
  Eigen::AngleAxisf aa(1.7F, Vec3::Random().normalized());
  EXPECT_TRUE(
    ExpectEqMat3(aa.toRotationMatrix(), AxisAngleToDCM(aa.angle(), aa.axis())));
}

TEST(AxisAngleToRotation, ThirdQuarterAngleRandomVector) {
  Eigen::AngleAxisf aa(-2.F, Vec3::Random().normalized());
  EXPECT_TRUE(
    ExpectEqMat3(aa.toRotationMatrix(), AxisAngleToDCM(aa.angle(), aa.axis())));
}

TEST(AxisAngleToRotation, FourthQuarterAngleRandomVector) {
  Eigen::AngleAxisf aa(-0.4F, Vec3::Random().normalized());
  EXPECT_TRUE(
    ExpectEqMat3(aa.toRotationMatrix(), AxisAngleToDCM(aa.angle(), aa.axis())));
}

TEST(Euler, XYZToRotationMatrix) {
  const RPY rpy(0.2F, -0.8F, 1.8F);
  const Mat3 rot = Mat3(Eigen::AngleAxisf(rpy.roll, Vec3::UnitX()) *
                        Eigen::AngleAxisf(rpy.pitch, Vec3::UnitY()) *
                        Eigen::AngleAxisf(rpy.yaw, Vec3::UnitZ()));
  EXPECT_TRUE(ExpectEqMat3(rot, EulerToDCM(rpy, EulerOrder::XYZ)));
}

TEST(Euler, ZYXToRotationMatrix) {
  const RPY rpy(0.2F, -0.8F, 1.8F);
  const Mat3 rot = Mat3(Eigen::AngleAxisf(rpy.yaw, Vec3::UnitZ()) *
                        Eigen::AngleAxisf(rpy.pitch, Vec3::UnitY()) *
                        Eigen::AngleAxisf(rpy.roll, Vec3::UnitX()));
  EXPECT_TRUE(ExpectEqMat3(rot, EulerToDCM(rpy, EulerOrder::ZYX)));
}

TEST(QuaternionToDCM, Insanity) {
  const Quat q = Quat::UnitRandom();
  const DCM dcm = QuaternionToDCM(q);
  EXPECT_TRUE(ExpectEqMat3(q.toRotationMatrix(), dcm.Matrix()));
}

TEST(AxisAngleToDCM, Insanity) {
  Eigen::AngleAxisf aa(0.2F, Vec3::Random().normalized());
  const DCM dcm = AxisAngleToDCM(aa.angle(), aa.axis());
  EXPECT_TRUE(ExpectEqMat3(aa.toRotationMatrix(), dcm.Matrix()));
}

TEST(EulerToDCM, Insanity1) {
  const RPY rpy(0.2F, -1.3F, 2.3F);
  const DCM expect = EulerToDCM(rpy, EulerOrder::XYZ);
  const RPY actual_rpy = DCMToEuler(expect, EulerOrder::XYZ);
  EXPECT_TRUE(ExpectEqRPY(rpy, actual_rpy));
}

TEST(EulerToDCM, Insanity2) {
  const RPY rpy(0.8F, 1.3F, -2.3F);
  const DCM expect = EulerToDCM(rpy, EulerOrder::ZYX);
  const RPY actual_rpy = DCMToEuler(expect, EulerOrder::ZYX);
  EXPECT_TRUE(ExpectEqRPY(rpy, actual_rpy));
}

TEST(DCMToEuler, QuaternionToEulerXYZ) {
  const Mat3 rot = RandomRotationMatrix();
  const DCM dcm(rot);
  const RPY actual_rpy = DCMToEuler(dcm, EulerOrder::XYZ);
  // Not working due to non unique solution, so check rotation instead
  // const auto expect_angles = rot.eulerAngles(0, 1, 2);
  // EXPECT_FLOAT_EQ(expect_angles.x(), actual_rpy.roll);
  // EXPECT_FLOAT_EQ(expect_angles.y(), actual_rpy.pitch);
  // EXPECT_FLOAT_EQ(expect_angles.z(), actual_rpy.yaw);
  EXPECT_TRUE(ExpectEqMat3(rot, EulerToDCM(actual_rpy, EulerOrder::XYZ)));
}

TEST(DCMToEuler, QuaternionToEulerZYX) {
  const Mat3 rot = RandomRotationMatrix();
  const DCM dcm(rot);
  const RPY actual_rpy = DCMToEuler(dcm, EulerOrder::ZYX);
  // Not working due to non unique solution, so check rotation instead
  // const auto expect_angles = rot.eulerAngles(2, 1, 0);
  // EXPECT_FLOAT_EQ(expect_angles.x(), actual_angles.roll);
  // EXPECT_FLOAT_EQ(expect_angles.y(), actual_angles.pitch);
  // EXPECT_FLOAT_EQ(expect_angles.z(), actual_angles.yaw);
  EXPECT_TRUE(ExpectEqMat3(rot, EulerToDCM(actual_rpy, EulerOrder::ZYX)));
}

TEST(EulerToDCM, EulerXYZ) {
  const RPY expect_rpy(0.3F, 0.5F, -0.8F);
  const DCM dcm = EulerToDCM(expect_rpy, EulerOrder::ZYX);
  const RPY actual_rpy = DCMToEuler(dcm, EulerOrder::ZYX);
  EXPECT_TRUE(ExpectEqRPY(expect_rpy, actual_rpy));
}

TEST(EulerToDCM, EulerZYX) {
  const RPY expect_rpy(1.1F, -0.7F, 0.7F);
  const DCM dcm = EulerToDCM(expect_rpy, EulerOrder::ZYX);
  const RPY actual_rpy = DCMToEuler(dcm, EulerOrder::ZYX);
  EXPECT_TRUE(ExpectEqRPY(expect_rpy, actual_rpy));
}

TEST(QuaternionToDCM, Random) {
  for (size_t i = 0; i < 1000; i++) {
    const Quat q = Quat::UnitRandom();
    const Mat3 expect = q.toRotationMatrix();
    const DCM actual = QuaternionToDCM(q);
    EXPECT_TRUE(ExpectEqMat3(expect, actual.Matrix()));
  }
}

TEST(DCMToQuaternion, ToQuaternion) {
  for (size_t i = 0; i < 1000; i++) {
    const Quat expect = Quat::UnitRandom();
    const DCM dcm = QuaternionToDCM(expect);
    const Quaternion actual = DCMToQuaternion(dcm, QuaternionMethod::SHEPPERD);
    EXPECT_TRUE(ExpectEqQuaternion(expect, actual));
  }
}

/*****************************************************************************/
class QuaternionAndAxisAngleFixture
  : public ::testing::TestWithParam<std::tuple<float, Vec3>> {};

TEST_P(QuaternionAndAxisAngleFixture, AxisAngleToQuaternion) {
  const auto [angle, axis] = GetParam();
  const Eigen::Quaternionf expect(Eigen::AngleAxisf(angle, axis));
  const Quaternion actual = AxisAngleToQuaternion(angle, axis);
  EXPECT_TRUE(ExpectEqQuaternion(expect, actual));
}

TEST_P(QuaternionAndAxisAngleFixture, QuaternionToAxisAngle) {
  const auto [angle, axis] = GetParam();
  const Eigen::Quaternionf expect(Eigen::AngleAxisf(angle, axis));
  const auto [actual_ang, actual_axis] = QuaternionToAxisAngle(expect);

  //
  float sign = 1.F;
  if (angle * actual_ang < 0) {
    sign = -1.F;
  }

  EXPECT_TRUE(ExpectEq(sign * angle, actual_ang));
  EXPECT_TRUE(ExpectEqVec3(sign * axis, actual_axis));
}

INSTANTIATE_TEST_CASE_P(
  QuaternionAndAxisAngle, QuaternionAndAxisAngleFixture,
  ::testing::Values(std::make_tuple(+core::math::PI_2, Vec3::UnitX()),
                    std::make_tuple(-core::math::PI_2, Vec3::UnitX()),
                    std::make_tuple(+core::math::PI_2, Vec3::UnitY()),
                    std::make_tuple(-core::math::PI_2, Vec3::UnitY()),
                    std::make_tuple(+core::math::PI_2, Vec3::UnitZ()),
                    std::make_tuple(-core::math::PI_2, Vec3::UnitZ()),
                    std::make_tuple(+core::math::PI_2 * 3, Vec3::UnitX()),
                    std::make_tuple(-core::math::PI_2 * 3, Vec3::UnitX()),
                    std::make_tuple(+core::math::PI_2 * 3, Vec3::UnitY()),
                    std::make_tuple(-core::math::PI_2 * 3, Vec3::UnitY()),
                    std::make_tuple(+core::math::PI_2 * 3, Vec3::UnitZ()),
                    std::make_tuple(-core::math::PI_2 * 3, Vec3::UnitZ()),
                    std::make_tuple(+core::math::PI_2, Vec3::UnitX() * -1),
                    std::make_tuple(-core::math::PI_2, Vec3::UnitX() * -1),
                    std::make_tuple(+core::math::PI_2, Vec3::UnitY() * -1),
                    std::make_tuple(-core::math::PI_2, Vec3::UnitY() * -1),
                    std::make_tuple(+core::math::PI_2, Vec3::UnitZ() * -1),
                    std::make_tuple(-core::math::PI_2, Vec3::UnitZ() * -1),
                    std::make_tuple(+core::math::PI_2 * 3, Vec3::UnitX() * -1),
                    std::make_tuple(-core::math::PI_2 * 3, Vec3::UnitX() * -1),
                    std::make_tuple(+core::math::PI_2 * 3, Vec3::UnitY() * -1),
                    std::make_tuple(-core::math::PI_2 * 3, Vec3::UnitY() * -1),
                    std::make_tuple(+core::math::PI_2 * 3, Vec3::UnitZ() * -1),
                    std::make_tuple(-core::math::PI_2 * 3,
                                    Vec3::UnitZ() * -1)));
