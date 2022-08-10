#include <gtest/gtest.h>

#include "core/utils/data.hpp"
#include "utest_data.hpp"

using namespace core::utils;

// constexpr auto UndefinedType = core::utils::ModuleType::UNDEFINED;

TEST(Vec3, DefaultValuesUsingBrackets)
{
  Vec3 vec = Vec3::Zero();
  EXPECT_FLOAT_EQ(0, vec[0]);
  EXPECT_FLOAT_EQ(0, vec[1]);
  EXPECT_FLOAT_EQ(0, vec[2]);
}

TEST(Vec3, DefaultValuesUsingNames)
{
  const Vec3 vec = Vec3::Zero();
  EXPECT_FLOAT_EQ(0, vec.x());
  EXPECT_FLOAT_EQ(0, vec.y());
  EXPECT_FLOAT_EQ(0, vec.z());
}

TEST(Vec3, Construct)
{
  const Vec3 vec(1, 2, 3);
  EXPECT_FLOAT_EQ(1, vec[0]);
  EXPECT_FLOAT_EQ(2, vec[1]);
  EXPECT_FLOAT_EQ(3, vec[2]);
}

TEST(Vec3, ConstructInRowStyle)
{
  const float arr[] = {4, 5, 6};
  Vec3 vec;
  vec << arr[0], arr[1], arr[2];
  EXPECT_FLOAT_EQ(arr[0], vec[0]);
  EXPECT_FLOAT_EQ(arr[1], vec[1]);
  EXPECT_FLOAT_EQ(arr[2], vec[2]);
}

TEST(Vec3, RefernceValue)
{
  Vec3 vec;
  vec.x() = 1;
  EXPECT_FLOAT_EQ(1, vec.x());
}

TEST(Quat, DefaultValuesUsingNames)
{
  const Quat quat = Quat::Identity();
  EXPECT_FLOAT_EQ(1, quat.w());
  EXPECT_FLOAT_EQ(0, quat.x());
  EXPECT_FLOAT_EQ(0, quat.y());
  EXPECT_FLOAT_EQ(0, quat.z());
}

TEST(ImuData, Consruct)
{
  const ImuData imu;
  ExpectImuData(ImuData(), imu);
}

TEST(ImuData, Clear)
{
  ImuData imu;
  imu.gyro.x() = 1;
  EXPECT_FLOAT_EQ(1, imu.gyro.x());
  imu.Clear();
  EXPECT_FLOAT_EQ(0, imu.gyro.x());
}