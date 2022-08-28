#ifndef TEST_UTEST_DATA_HPP
#define TEST_UTEST_DATA_HPP

#include <sstream>

#include "core/utils/math.hpp"
#include "gtest/gtest.h"
#include "utest_data.hpp"

using namespace core::utils;

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