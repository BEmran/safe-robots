// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/math.hpp"
#include "my_math/dcm.hpp"

using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::Quat;
using core::utils::RPY;
using core::utils::Vec3;

constexpr float EPS = 0.00001f;
[[nodiscard]] ::testing::AssertionResult ExpectEqMat3(const Mat3& expect,
                                                      const Mat3& actual) {
  for (Eigen::Index i = 0; i < expect.size(); i++) {
    if (std::abs(expect(i) - actual(i)) > EPS) {
      const long row = i / expect.cols();
      const long col = i % expect.cols();
      return ::testing::AssertionFailure()
             << "idx(" << i << ") at Mat[" << row << "," << col << "] -> "
             << "expect (" << expect(i) << ") and actual (" << actual(i) << ")";
    }
  }
  return ::testing::AssertionSuccess();
}

[[nodiscard]] ::testing::AssertionResult ExpectEqVec3(const Vec3& expect,
                                                      const Vec3& actual) {
  for (Eigen::Index i = 0; i < expect.size(); i++) {
    if (std::abs(expect(i) - actual(i)) > EPS) {
      return ::testing::AssertionFailure()
             << "idx(" << i << ") -> "
             << "expect (" << expect(i) << ") and actual (" << actual(i) << ")";
    }
  }
  return ::testing::AssertionSuccess();
}

[[nodiscard]] ::testing::AssertionResult ExpectEqQuat(const Quat& expect,
                                                      const Quat& actual) {
  // std::cout << expect << std::endl;
  // std::cout << actual << std::endl;
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

TEST(DCM, DefaultConstructor) {
  const DCM dcm;
  EXPECT_TRUE(ExpectEqMat3(Mat3::Identity(), dcm.Matrix()));
}

TEST(DCM, ConstructFromQuat) {
  const Quat q = Quat::UnitRandom();
  const DCM dcm(q);
  EXPECT_TRUE(ExpectEqMat3(q.toRotationMatrix(), dcm.Matrix()));
}

TEST(DCM, ConstructFromAngleAxis) {
  Eigen::AngleAxisf aa(0.2f, Vec3::Random().normalized());
  const DCM dcm(aa.angle(), aa.axis());
  EXPECT_TRUE(ExpectEqMat3(aa.toRotationMatrix(), dcm.Matrix()));
}

TEST(DCM, ConstructFromEulerAngleXYZ) {
  const RPY rpy(0.2f, -1.3f, 2.3f);
  const Mat3 rot = EulerToDCM(rpy, EulerOrder::XYZ);
  const DCM dcm(rpy, EulerOrder::XYZ);
  EXPECT_TRUE(ExpectEqMat3(rot, dcm.Matrix()));
}

TEST(DCM, ConstructFromEulerAngleZYX) {
  const RPY rpy(0.8f, 1.3f, -2.3f);
  const Mat3 rot = EulerToDCM(rpy, EulerOrder::ZYX);
  const DCM dcm(rpy, EulerOrder::ZYX);
  EXPECT_TRUE(ExpectEqMat3(rot, dcm.Matrix()));
}

TEST(DCM, QuaternionToEulerXYZ) {
  const Mat3 rot = Quat::UnitRandom().toRotationMatrix();
  const auto expect_angles = rot.eulerAngles(0, 1, 2);
  const DCM dcm(rot);
  const RPY actual_angles = dcm.ToEuler(EulerOrder::XYZ);
  EXPECT_FLOAT_EQ(expect_angles.x(), actual_angles.roll);
  EXPECT_FLOAT_EQ(expect_angles.y(), actual_angles.pitch);
  EXPECT_FLOAT_EQ(expect_angles.z(), actual_angles.yaw);
}

TEST(DCM, QuaternionToEulerZYX) {
  const Mat3 rot = Quat::UnitRandom().toRotationMatrix();
  // const auto expect_angles = rot.eulerAngles(2, 1, 0);
  const DCM dcm(rot);
  const RPY actual_angles = dcm.ToEuler(EulerOrder::ZYX);
  // Not working due to non unique solution, so check rotation instead
  // EXPECT_FLOAT_EQ(expect_angles.x(), actual_angles.roll);
  // EXPECT_FLOAT_EQ(expect_angles.y(), actual_angles.pitch);
  // EXPECT_FLOAT_EQ(expect_angles.z(), actual_angles.yaw);
  EXPECT_TRUE(ExpectEqMat3(rot, EulerToDCM(actual_angles, EulerOrder::ZYX)));
}

TEST(DCM, GetEulerXYZ) {
  const RPY expect_rpy(0.3f, 0.5f, -0.8f);
  const DCM dcm(expect_rpy, EulerOrder::ZYX);
  const RPY actual_angles = dcm.ToEuler(EulerOrder::ZYX);
  EXPECT_FLOAT_EQ(expect_rpy.roll, actual_angles.roll);
  EXPECT_FLOAT_EQ(expect_rpy.pitch, actual_angles.pitch);
  EXPECT_FLOAT_EQ(expect_rpy.yaw, actual_angles.yaw);
}

TEST(DCM, GetEulerZYX) {
  const RPY expect_rpy(1.1f, -0.7f, 0.7f);
  const DCM dcm(expect_rpy, EulerOrder::ZYX);
  const RPY actual_angles = dcm.ToEuler(EulerOrder::ZYX);
  EXPECT_FLOAT_EQ(expect_rpy.roll, actual_angles.roll);
  EXPECT_FLOAT_EQ(expect_rpy.pitch, actual_angles.pitch);
  EXPECT_FLOAT_EQ(expect_rpy.yaw, actual_angles.yaw);
}

TEST(DCM, ToQuaternion) {
  for (size_t i = 0; i < 1000; i++) {
    Quat expect_quat = Quat::UnitRandom();
    DCM dcm(expect_quat);
    const Quat actual_quat = dcm.ToQuaternion(QuaternionMethod::SHEPPERD);
    EXPECT_TRUE(ExpectEqQuat(expect_quat, actual_quat));
  }
}