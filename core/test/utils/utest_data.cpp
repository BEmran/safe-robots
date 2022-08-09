#include <gtest/gtest.h>

#include "core/utils/data.hpp"
#include "utest_data.hpp"

using namespace core::utils;

// constexpr auto UndefinedType = core::utils::ModuleType::UNDEFINED;

TEST(Vec3, DefaultValuesUsingBrackets)
{
  Vec3 vec;
  EXPECT_DOUBLE_EQ(0, vec[0]);
  EXPECT_DOUBLE_EQ(0, vec[1]);
  EXPECT_DOUBLE_EQ(0, vec[2]);
}

TEST(Vec3, DefaultValuesUsingNames)
{
  const Vec3 vec;
  EXPECT_DOUBLE_EQ(0, vec.x());
  EXPECT_DOUBLE_EQ(0, vec.y());
  EXPECT_DOUBLE_EQ(0, vec.z());
}

TEST(Vec3, Construct)
{
  const Vec3 vec(1,2,3);
  EXPECT_DOUBLE_EQ(1, vec[0]);
  EXPECT_DOUBLE_EQ(2, vec[1]);
  EXPECT_DOUBLE_EQ(3, vec[2]);
}

TEST(Vec3, ConstructUsingArray)
{
  const double arr[] = {4,5,6};
  const Vec3 vec(arr);
  EXPECT_DOUBLE_EQ(arr[0], vec[0]);
  EXPECT_DOUBLE_EQ(arr[1], vec[1]);
  EXPECT_DOUBLE_EQ(arr[2], vec[2]);
}

// TEST(Vec3, ConstructWithWrongSizeArray)
// {
//   const double arr[] = {4,5};
//   Vec3 a(arr);
// }

TEST(Vec3, RefernceValue)
{
  Vec3 vec;
  vec.x() = 1;
  EXPECT_DOUBLE_EQ(1, vec.x());
}

TEST(Quat, DefaultValuesUsingNames)
{
  const Quat quat;
  EXPECT_DOUBLE_EQ(1, quat.angle);
  EXPECT_DOUBLE_EQ(0, quat.vector[0]);
  EXPECT_DOUBLE_EQ(0, quat.vector[1]);
  EXPECT_DOUBLE_EQ(0, quat.vector[2]);
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
  EXPECT_DOUBLE_EQ(1, imu.gyro.x());
  imu.Clear();
  EXPECT_DOUBLE_EQ(0, imu.gyro.x());
}