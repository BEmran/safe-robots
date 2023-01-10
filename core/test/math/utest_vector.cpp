// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <iostream>
#include <sstream>
#include <string>
#include <string_view>

#include "core/math/vector.hpp"
#include "core/utils/clock.hpp"
#include "core/math/math.hpp"
#include "math/quaternion.h"

using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::Quat;
using core::utils::Vec3;

std::string ToString(const Quat quat) {
  std::stringstream ss;
  ss << "w: " << quat.w()   //
     << " x: " << quat.x()  //
     << " y: " << quat.y()  //
     << " z: " << quat.z();
  return ss.str();
}

template <typename T>
std::string ToString(const math::Vec<T> vec) {
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < vec.Length(); i++) {
    ss << vec[i] << " ";
  }
  ss << "]";
  return ss.str();
}

std::string ToString(const rc_vector_t vec) {
  if (vec.len != 4)
    return "";
  std::stringstream ss;
  ss << "w: " << vec.d[0]   //
     << " x: " << vec.d[1]  //
     << " y: " << vec.d[2]  //
     << " z: " << vec.d[3];
  return ss.str();
}

void Print(std::string_view str, const Quat quat) {
  std::cout << str << ": " << ToString(quat) << std::endl;
}

void Print(std::string_view str, const rc_vector_t quat) {
  std::cout << str << ": " << ToString(quat) << std::endl;
}

template <typename T>
void Print(std::string_view str, const math::Vec<T> vec) {
  std::cout << str << ": " << ToString(vec) << std::endl;
}

template <typename T>
::testing::AssertionResult ExpectEq(const std::vector<T> expect,
                                    const math::Vec<T>& actual) {
  if (expect.size() != actual.Length()) {
    return ::testing::AssertionFailure()
           << "size mismatch, expect: " << expect.size()
           << " and got: " << actual.Length();
  }
  for (size_t i = 0; i < expect.size(); i++) {
    EXPECT_DOUBLE_EQ(expect[i], actual[i]);
  }
  return ::testing::AssertionSuccess();
}

TEST(Vec, DefaultConstructor) {
  math::Vec<double> vec;
  ASSERT_EQ(0, vec.Length());
  ASSERT_TRUE(vec.IsEmpty());
}

TEST(Vec, InitializeListConstructor) {
  math::Vec<double> vec{1., 0.2};
  EXPECT_TRUE(ExpectEq({1., 0.2}, vec));
}

TEST(Vec, InitializeWithSizeConstructor) {
  math::Vec<double> vec(1., 4);
  EXPECT_TRUE(ExpectEq({1., 1., 1., 1.}, vec));
}

TEST(Vec, Norm) {
  math::Vec<double> vec{3., 4.};
  EXPECT_DOUBLE_EQ(5.0, vec.Norm());
}

TEST(Vec, NormOf1) {
  math::Vec<double> vec{1.};
  EXPECT_DOUBLE_EQ(1.0, vec.Norm());
}

TEST(Vec, Normalize) {
  math::Vec<double> vec{3., 4.};
  vec.Normalize();
  EXPECT_TRUE(ExpectEq({3 / 5., 4 / 5.}, vec));
}

TEST(Vec, Modify) {
  math::Vec<double> vec{1., 2};
  vec[0] = 4.;
  vec[1] = 5.;
  EXPECT_DOUBLE_EQ(4., vec[0]);
  EXPECT_DOUBLE_EQ(5., vec[1]);
}

TEST(CrossProduct, Fundamental) {
  math::Vec3<double> x{1., 0., 0.};
  math::Vec3<double> y{0., 1., 0.};
  math::Vec3<double> z{0., 0., 1.};
  EXPECT_TRUE(ExpectEq({0., 0., +1.}, x.Cross(y)));
  EXPECT_TRUE(ExpectEq({+1., 0., 0.}, y.Cross(z)));
  EXPECT_TRUE(ExpectEq({0., +1., 0.}, z.Cross(x)));
  EXPECT_TRUE(ExpectEq({0., -1., 0.}, x.Cross(z)));
  EXPECT_TRUE(ExpectEq({0., 0., -1.}, y.Cross(x)));
  EXPECT_TRUE(ExpectEq({-1., 0., 0.}, z.Cross(y)));
  EXPECT_TRUE(ExpectEq({0., 0., 0.}, x.Cross(x)));
  EXPECT_TRUE(ExpectEq({0., 0., 0.}, y.Cross(y)));
  EXPECT_TRUE(ExpectEq({0., 0., 0.}, z.Cross(z)));
}

TEST(DotProduct, Fundamental) {
  math::Vec3<double> x{1., 2., 3.};
  math::Vec3<double> y{4., 1., 2.};
  EXPECT_DOUBLE_EQ(12., x.Dot(y));
}

TEST(Vec3, DefaultConstructor) {
  math::Vec3<double> vec;
  ASSERT_EQ(3, vec.Length());
  EXPECT_TRUE(ExpectEq({0., 0., 0.}, vec));
}

TEST(Vec3, ParameterConstructor) {
  math::Vec3<double> vec{1., 2., 3.};
  EXPECT_TRUE(ExpectEq({1., 2., 3.}, vec));
}

TEST(Vec3, InitializeListConstructor) {
  math::Vec3<double> vec{{1., 2., 3.}};
  EXPECT_TRUE(ExpectEq({1., 2., 3.}, vec));
}

TEST(Vec3, Norm) {
  math::Vec3<double> vec{3., 0., 4.};
  EXPECT_DOUBLE_EQ(5.0, vec.Norm());
}

TEST(Vec3, Normalize) {
  math::Vec3<double> vec{0., 3., 4.};
  vec.Normalize();
  EXPECT_TRUE(ExpectEq({0., 3 / 5., 4 / 5.}, vec));
}

TEST(Vec3, Cross) {
  math::Vec3<double> x{1., 0., 0.};
  math::Vec3<double> y{0., 1., 0.};
  math::Vec3<double> z{0., 0., 1.};
  EXPECT_TRUE(ExpectEq({0., 0., +1.}, x.Cross(y)));
  EXPECT_TRUE(ExpectEq({0., -1., 0.}, x.Cross(z)));
  EXPECT_TRUE(ExpectEq({+1., 0., 0.}, y.Cross(z)));
  EXPECT_TRUE(ExpectEq({0., 0., -1.}, y.Cross(x)));
  EXPECT_TRUE(ExpectEq({0., +1., 0.}, z.Cross(x)));
  EXPECT_TRUE(ExpectEq({-1., 0., 0.}, z.Cross(y)));
}

TEST(Vec3, Dot) {
  math::Vec3<double> x{1., 2., 3.};
  math::Vec3<double> y{4., 1., 2.};
  EXPECT_DOUBLE_EQ(12., x.Dot(y));
  EXPECT_DOUBLE_EQ(12., y.Dot(x));
  EXPECT_DOUBLE_EQ(x.Norm() * x.Norm(), x.Dot(x));
  EXPECT_DOUBLE_EQ(y.Norm() * y.Norm(), y.Dot(y));
}

TEST(DotProduct, XYZ) {
  math::Vec3<double> vec{1., 2., 3.};
  EXPECT_DOUBLE_EQ(vec[0], vec.X());
  EXPECT_DOUBLE_EQ(vec[1], vec.Y());
  EXPECT_DOUBLE_EQ(vec[2], vec.Z());
}

TEST(DotProduct, ModifyXYZ) {
  math::Vec3<double> vec{1., 2., 3.};
  vec.X() = 4.;
  vec.Y() = 5.;
  vec.Z() = 6.;
  EXPECT_DOUBLE_EQ(4., vec.X());
  EXPECT_DOUBLE_EQ(5., vec.Y());
  EXPECT_DOUBLE_EQ(6., vec.Z());
}