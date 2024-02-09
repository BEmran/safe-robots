// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include <iostream>
#include <vector>

#include "core/math/vector.hpp"

using core::math::my::Vec;
using core::math::my::Vec3;

template <typename T>
::testing::AssertionResult ExpectEqStdVector(const std::vector<T> expect,
                                             const Vec<T>& actual) {
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
  Vec<double> vec;
  ASSERT_EQ(0, vec.Length());
  ASSERT_TRUE(vec.IsEmpty());
}

TEST(Vec, InitializeListConstructor) {
  Vec<double> vec{1., 0.2};
  EXPECT_TRUE(ExpectEqStdVector({1., 0.2}, vec));
}

TEST(Vec, InitializeWithSizeConstructor) {
  Vec<double> vec(1., 4);
  EXPECT_TRUE(ExpectEqStdVector({1., 1., 1., 1.}, vec));
}

TEST(Vec, Norm) {
  Vec<double> vec{3., 4.};
  EXPECT_DOUBLE_EQ(5.0, vec.Norm());
}

TEST(Vec, NormOf1) {
  Vec<double> vec{1.};
  EXPECT_DOUBLE_EQ(1.0, vec.Norm());
}

TEST(Vec, Normalize) {
  Vec<double> vec{3., 4.};
  vec.Normalize();
  EXPECT_TRUE(ExpectEqStdVector({3 / 5., 4 / 5.}, vec));
}

TEST(Vec, Modify) {
  Vec<double> vec{1., 2};
  vec[0] = 4.;
  vec[1] = 5.;
  EXPECT_DOUBLE_EQ(4., vec[0]);
  EXPECT_DOUBLE_EQ(5., vec[1]);
}

TEST(CrossProduct, Fundamental) {
  Vec3<double> x{1., 0., 0.};
  Vec3<double> y{0., 1., 0.};
  Vec3<double> z{0., 0., 1.};
  EXPECT_TRUE(ExpectEqStdVector({0., 0., +1.}, x.Cross(y)));
  EXPECT_TRUE(ExpectEqStdVector({+1., 0., 0.}, y.Cross(z)));
  EXPECT_TRUE(ExpectEqStdVector({0., +1., 0.}, z.Cross(x)));
  EXPECT_TRUE(ExpectEqStdVector({0., -1., 0.}, x.Cross(z)));
  EXPECT_TRUE(ExpectEqStdVector({0., 0., -1.}, y.Cross(x)));
  EXPECT_TRUE(ExpectEqStdVector({-1., 0., 0.}, z.Cross(y)));
  EXPECT_TRUE(ExpectEqStdVector({0., 0., 0.}, x.Cross(x)));
  EXPECT_TRUE(ExpectEqStdVector({0., 0., 0.}, y.Cross(y)));
  EXPECT_TRUE(ExpectEqStdVector({0., 0., 0.}, z.Cross(z)));
}

TEST(DotProduct, Fundamental) {
  Vec3<double> x{1., 2., 3.};
  Vec3<double> y{4., 1., 2.};
  EXPECT_DOUBLE_EQ(12., x.Dot(y));
}

TEST(Vec3, DefaultConstructor) {
  Vec3<double> vec;
  ASSERT_EQ(3, vec.Length());
  EXPECT_TRUE(ExpectEqStdVector({0., 0., 0.}, vec));
}

TEST(Vec3, ParameterConstructor) {
  Vec3<double> vec{1., 2., 3.};
  EXPECT_TRUE(ExpectEqStdVector({1., 2., 3.}, vec));
}

TEST(Vec3, InitializeListConstructor) {
  Vec3<double> vec{{1., 2., 3.}};
  EXPECT_TRUE(ExpectEqStdVector({1., 2., 3.}, vec));
}

TEST(Vec3, Norm) {
  Vec3<double> vec{3., 0., 4.};
  EXPECT_DOUBLE_EQ(5.0, vec.Norm());
}

TEST(Vec3, Normalize) {
  Vec3<double> vec{0., 3., 4.};
  vec.Normalize();
  EXPECT_TRUE(ExpectEqStdVector({0., 3 / 5., 4 / 5.}, vec));
}

TEST(Vec3, Cross) {
  Vec3<double> x{1., 0., 0.};
  Vec3<double> y{0., 1., 0.};
  Vec3<double> z{0., 0., 1.};
  EXPECT_TRUE(ExpectEqStdVector({0., 0., +1.}, x.Cross(y)));
  EXPECT_TRUE(ExpectEqStdVector({0., -1., 0.}, x.Cross(z)));
  EXPECT_TRUE(ExpectEqStdVector({+1., 0., 0.}, y.Cross(z)));
  EXPECT_TRUE(ExpectEqStdVector({0., 0., -1.}, y.Cross(x)));
  EXPECT_TRUE(ExpectEqStdVector({0., +1., 0.}, z.Cross(x)));
  EXPECT_TRUE(ExpectEqStdVector({-1., 0., 0.}, z.Cross(y)));
}

TEST(Vec3, Dot) {
  Vec3<double> x{1., 2., 3.};
  Vec3<double> y{4., 1., 2.};
  EXPECT_DOUBLE_EQ(12., x.Dot(y));
  EXPECT_DOUBLE_EQ(12., y.Dot(x));
  EXPECT_DOUBLE_EQ(x.Norm() * x.Norm(), x.Dot(x));
  EXPECT_DOUBLE_EQ(y.Norm() * y.Norm(), y.Dot(y));
}

TEST(DotProduct, XYZ) {
  Vec3<double> vec{1., 2., 3.};
  EXPECT_DOUBLE_EQ(vec[0], vec.X());
  EXPECT_DOUBLE_EQ(vec[1], vec.Y());
  EXPECT_DOUBLE_EQ(vec[2], vec.Z());
}

TEST(DotProduct, ModifyXYZ) {
  Vec3<double> vec{1., 2., 3.};
  vec.X() = 4.;
  vec.Y() = 5.;
  vec.Z() = 6.;
  EXPECT_DOUBLE_EQ(4., vec.X());
  EXPECT_DOUBLE_EQ(5., vec.Y());
  EXPECT_DOUBLE_EQ(6., vec.Z());
}