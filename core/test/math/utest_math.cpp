// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef TEST_UTEST_DATA_HPP
#define TEST_UTEST_DATA_HPP

#include <gtest/gtest.h>

#include <sstream>

#include "core/math/math.hpp"
#include "utest/utils_data.hpp"

using core::utils::InputMat;
using core::utils::Matrix;
using core::utils::MatrixX;
using core::utils::OutputMat;
using core::utils::Scalar;
using core::utils::Vector;

float Sum(InputMat mat) {
  return mat.sum();
}

void Initialize(int size, MATH_TYPE* arr, OutputMat mat) {
  std::copy(arr, arr + size, mat.data());
}
// General --------------------------------------------------------------------

TEST(InputMat, PassingVectorToFuncAsInput) {
  constexpr auto size = 5;
  Vector<size> vec;
  MATH_TYPE arr[size] = {1, 2, 3, 1, 4};
  std::copy(arr, arr + size, vec.data());

  ASSERT_EQ(size, vec.size());
  EXPECT_EQ(Sum(vec), vec.sum());
}

TEST(InputMat, PassingMatrixToFuncAsInput) {
  constexpr auto size = 3;
  Matrix<size, size> mat;
  mat.setZero();
  MATH_TYPE arr[3] = {1, 2, 3};
  std::copy(arr, arr + size, mat.data());

  ASSERT_EQ(size * size, mat.size());
  EXPECT_EQ(Sum(mat), mat.sum());
}

TEST(InputMat, PassingMatrixToFuncAsOutput) {
  constexpr auto size = 2;
  MATH_TYPE arr[size * size] = {1, 2, 3, 1};

  Matrix<size, size> mat;
  ASSERT_EQ(size * size, mat.size());
  Initialize(size, arr, mat);
  EXPECT_TRUE(std::equal(arr, arr + size, mat.data()));
}

TEST(CreateVector, Vec4) {
  constexpr auto size = 4;
  Vector<size> vec;
  MATH_TYPE arr[size] = {1, 2, 3, 1};
  std::copy(arr, arr + size, vec.data());

  ASSERT_EQ(size, vec.size());
  EXPECT_TRUE(std::equal(arr, arr + size, vec.data()));
}

TEST(Scalar, Scalar) {
  Scalar scalar;
  scalar(0) = 1;
  ASSERT_EQ(1, scalar.size());
  ExpectEq(static_cast<MATH_TYPE>(1), scalar[0]);
}

TEST(Scalar, CreateScalar) {
  const auto scalar = core::utils::CreateScalar(3);
  ASSERT_EQ(1, scalar.size());
  ExpectEq(static_cast<MATH_TYPE>(3), scalar[0]);
}

TEST(CreateMatrix, Mat22) {
  Matrix<2, 2> mat;
  mat.setIdentity();
  constexpr auto size = 4;
  MATH_TYPE arr[size] = {1, 0, 0, 1};

  ASSERT_EQ(size, mat.size());
  EXPECT_TRUE(std::equal(arr, arr + size, mat.data()));
}
// Vec3 -----------------------------------------------------------------------
TEST(Vec3, DefaultValuesUsingBrackets) {
  Vec3 vec = Vec3::Zero();
  EXPECT_FLOAT_EQ(0, vec[0]);
  EXPECT_FLOAT_EQ(0, vec[1]);
  EXPECT_FLOAT_EQ(0, vec[2]);
}

TEST(Vec3, DefaultValuesUsingNames) {
  const Vec3 vec = Vec3::Zero();
  EXPECT_FLOAT_EQ(0, vec.x());
  EXPECT_FLOAT_EQ(0, vec.y());
  EXPECT_FLOAT_EQ(0, vec.z());
}

TEST(Vec3, Construct) {
  const Vec3 vec(1.0F, 2.0F, 3.0F);
  EXPECT_FLOAT_EQ(1, vec[0]);
  EXPECT_FLOAT_EQ(2, vec[1]);
  EXPECT_FLOAT_EQ(3, vec[2]);
}

TEST(Vec3, ConstructInRowStyle) {
  const float arr[] = {4.0F, 5.0F, 6.0F};
  Vec3 vec;
  vec << arr[0], arr[1], arr[2];
  EXPECT_FLOAT_EQ(arr[0], vec[0]);
  EXPECT_FLOAT_EQ(arr[1], vec[1]);
  EXPECT_FLOAT_EQ(arr[2], vec[2]);
}

TEST(Vec3, RefernceValue) {
  Vec3 vec;
  vec.x() = 1.0F;
  EXPECT_FLOAT_EQ(1.0F, vec.x());
}

TEST(Vec3, Print) {
  const Vec3 vec(1.4F, 2.5F, 3.6F);
  std::stringstream ss;
  ss << vec.transpose();
  EXPECT_EQ("[1.4, 2.5, 3.6]", ss.str());
}

// Quat -----------------------------------------------------------------------

TEST(Quat, DefaultValuesUsingNames) {
  const Quat quat = Quat::Identity();
  EXPECT_FLOAT_EQ(1, quat.w());
  EXPECT_FLOAT_EQ(0, quat.x());
  EXPECT_FLOAT_EQ(0, quat.y());
  EXPECT_FLOAT_EQ(0, quat.z());
}

TEST(Quat, DefaultValuesUsingVec) {
  const Quat quat = Quat::Identity();
  EXPECT_FLOAT_EQ(0, quat.vec().x());
  EXPECT_FLOAT_EQ(0, quat.vec().y());
  EXPECT_FLOAT_EQ(0, quat.vec().z());
}

TEST(Quat, Print) {
  Quat quat = Quat::Identity();
  quat.w() = 4.7F;
  quat.vec() = Vec3(1.4F, 2.5F, 3.6F);
  std::stringstream ss;
  ss << quat;
  EXPECT_EQ("ang = 4.7, [1.4, 2.5, 3.6]", ss.str());
}
#endif  // TEST_UTEST_DATA_HPP
