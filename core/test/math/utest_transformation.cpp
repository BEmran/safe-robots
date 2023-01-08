// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/math.hpp"
#include "my_math/dcm.hpp"
#include "my_math/transformation.hpp"
#include "utest_utils.hpp"

using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::Quat;
using core::utils::RPY;
using core::utils::Vec3;
using my::AxisAngleToDCM;
using my::DCM;
using my::DCMToAxisAngle;
using my::DCMToEuler;
using my::DCMToQuaternion;
using my::EulerOrder;
using my::EulerToDCM;
using my::QuaternionMethod;
using my::QuatToDCM;
using my::Skew;

TEST(Skew, SkewSymmetric) {
  const Vec3 vec(0.2f, -0.4f, 0.8f);
  Mat3 s = Skew(vec);
  EXPECT_TRUE(ExpectEqMat3(Mat3::Zero(), s + s.transpose()));
}

TEST(Skew, SkewSymmetric2) {
  const Vec3 vec(0.5f, -0.2f, 0.1f);
  Mat3 s = Skew(vec);
  EXPECT_TRUE(ExpectEqMat3(-s, s.transpose()));
}

TEST(Skew, DiagonalAreZeros) {
  const Vec3 vec(0.5f, -0.2f, 0.1f);
  Mat3 s = Skew(vec);
  EXPECT_TRUE(s.diagonal().isZero());
}

TEST(Skew, CheckVectorValues) {
  const Vec3 vec(0.5f, -0.2f, 0.1f);
  Mat3 s = Skew(vec);
  EXPECT_FLOAT_EQ(vec.x(), s(2, 1));
  EXPECT_FLOAT_EQ(vec.y(), s(0, 2));
  EXPECT_FLOAT_EQ(vec.z(), s(1, 0));
}

TEST(QuatToRotation, Random) {
  const Quat q1 = Quat::UnitRandom();
  EXPECT_TRUE(ExpectEqMat3(q1.toRotationMatrix(), QuatToDCM(q1)));
  const Quat q2 = q1.conjugate();
  EXPECT_TRUE(ExpectEqMat3(q2.toRotationMatrix(), QuatToDCM(q2)));
}

TEST(AxisAngleToRotation, Quarter1AngleRandomVector) {
  Eigen::AngleAxisf aa(0.2f, Vec3::Random().normalized());
  EXPECT_TRUE(
    ExpectEqMat3(aa.toRotationMatrix(), AxisAngleToDCM(aa.angle(), aa.axis())));
}

TEST(AxisAngleToRotation, Quarter2AngleRandomVector) {
  Eigen::AngleAxisf aa(1.7f, Vec3::Random().normalized());
  EXPECT_TRUE(
    ExpectEqMat3(aa.toRotationMatrix(), AxisAngleToDCM(aa.angle(), aa.axis())));
}

TEST(AxisAngleToRotation, Quarter3AngleRandomVector) {
  Eigen::AngleAxisf aa(-2.f, Vec3::Random().normalized());
  EXPECT_TRUE(
    ExpectEqMat3(aa.toRotationMatrix(), AxisAngleToDCM(aa.angle(), aa.axis())));
}

TEST(AxisAngleToRotation, Quarter4AngleRandomVector) {
  Eigen::AngleAxisf aa(-0.4f, Vec3::Random().normalized());
  EXPECT_TRUE(
    ExpectEqMat3(aa.toRotationMatrix(), AxisAngleToDCM(aa.angle(), aa.axis())));
}

TEST(Euler, XYZToRotationMatrix) {
  const RPY rpy(0.2f, -0.8f, 1.8f);
  const Mat3 rot = Mat3(Eigen::AngleAxisf(rpy.roll, Vec3::UnitX()) *
                        Eigen::AngleAxisf(rpy.pitch, Vec3::UnitY()) *
                        Eigen::AngleAxisf(rpy.yaw, Vec3::UnitZ()));
  EXPECT_TRUE(ExpectEqMat3(rot, EulerToDCM(rpy, EulerOrder::XYZ)));
}

TEST(Euler, ZYXToRotationMatrix) {
  const RPY rpy(0.2f, -0.8f, 1.8f);
  const Mat3 rot = Mat3(Eigen::AngleAxisf(rpy.yaw, Vec3::UnitZ()) *
                        Eigen::AngleAxisf(rpy.pitch, Vec3::UnitY()) *
                        Eigen::AngleAxisf(rpy.roll, Vec3::UnitX()));
  EXPECT_TRUE(ExpectEqMat3(rot, EulerToDCM(rpy, EulerOrder::ZYX)));
}

TEST(QuatToDCM, Insanity) {
  const Quat q = Quat::UnitRandom();
  const DCM dcm = QuatToDCM(q);
  EXPECT_TRUE(ExpectEqMat3(q.toRotationMatrix(), dcm.Matrix()));
}

TEST(AxisAngleToDCM, Insanity) {
  Eigen::AngleAxisf aa(0.2f, Vec3::Random().normalized());
  const DCM dcm = AxisAngleToDCM(aa.angle(), aa.axis());
  EXPECT_TRUE(ExpectEqMat3(aa.toRotationMatrix(), dcm.Matrix()));
}

TEST(EulerToDCM, Insanity1) {
  const RPY rpy(0.2f, -1.3f, 2.3f);
  const DCM expect = EulerToDCM(rpy, EulerOrder::XYZ);
  const RPY actual_rpy = DCMToEuler(expect, EulerOrder::XYZ);
  EXPECT_TRUE(ExpectEqRPY(rpy, actual_rpy));
}

TEST(EulerToDCM, Insanity2) {
  const RPY rpy(0.8f, 1.3f, -2.3f);
  const DCM expect = EulerToDCM(rpy, EulerOrder::ZYX);
  const RPY actual_rpy = DCMToEuler(expect, EulerOrder::ZYX);
  EXPECT_TRUE(ExpectEqRPY(rpy, actual_rpy));
}

TEST(DCMToEuler, QuaternionToEulerXYZ) {
  const Mat3 rot = Quat::UnitRandom().toRotationMatrix();
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
  const Mat3 rot = Quat::UnitRandom().toRotationMatrix();
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
  const RPY expect_rpy(0.3f, 0.5f, -0.8f);
  const DCM dcm = EulerToDCM(expect_rpy, EulerOrder::ZYX);
  const RPY actual_rpy = DCMToEuler(dcm, EulerOrder::ZYX);
  EXPECT_TRUE(ExpectEqRPY(expect_rpy, actual_rpy));
}

TEST(EulerToDCM, EulerZYX) {
  const RPY expect_rpy(1.1f, -0.7f, 0.7f);
  const DCM dcm = EulerToDCM(expect_rpy, EulerOrder::ZYX);
  const RPY actual_rpy = DCMToEuler(dcm, EulerOrder::ZYX);
  EXPECT_TRUE(ExpectEqRPY(expect_rpy, actual_rpy));
}

TEST(QuatToDCM, ToQuaternion) {
  for (size_t i = 0; i < 1000; i++) {
    Quat expect_quat = Quat::UnitRandom();
    DCM dcm = QuatToDCM(expect_quat);
    const Quat actual_quat =
      my::DCMToQuaternion(dcm, QuaternionMethod::SHEPPERD);
    EXPECT_TRUE(ExpectEqQuat(expect_quat, actual_quat));
  }
}