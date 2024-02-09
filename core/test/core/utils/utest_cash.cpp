// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/cash.hpp"
#include "core/utils/data.hpp"
#include "utils_data.hpp"

using core::utils::Cash;

TEST(Cash, SetAndGet) {
  ImuData imu;
  Cash<ImuData> cash;
  imu.accel.data.x() = 1;
  imu.gyro.data.y() = 1;
  imu.mag.data.z() = 1;
  imu.quat.data.w() = 0.5;
  cash.Set(imu);
  EXPECT_TRUE(ExpectImuData(imu, cash.Get()));
}

TEST(Cash, SetAndClear) {
  ImuData imu;
  Cash<ImuData> cash;
  imu.accel.data.x() = 1;
  cash.Set(imu);
  EXPECT_DOUBLE_EQ(1, cash.Get().accel.data.x());
  cash.Clear();
  EXPECT_DOUBLE_EQ(0, cash.Get().accel.data.x());
}
