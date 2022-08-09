#include <gtest/gtest.h>

#include "core/utils/cash.hpp"
#include "core/utils/data.hpp"
#include "utest_data.hpp"

using namespace core::utils;

TEST(Cash, SetAndGet)
{
  ImuData imu;
  Cash<ImuData> cash;
  imu.accel.x() = 1;
  imu.gyro.y() = 1;
  imu.mag.z() = 1;
  imu.quat.angle = 0.5;
  cash.Set(imu);
  ExpectImuData(imu, cash.Get());
}

TEST(Cash, SetAndClear)
{
  ImuData imu;
  Cash<ImuData> cash;
  imu.accel.x() = 1;
  cash.Set(imu);
  EXPECT_DOUBLE_EQ(1, cash.Get().accel.x());
  cash.Clear();
  EXPECT_DOUBLE_EQ(0, cash.Get().accel.x());
}