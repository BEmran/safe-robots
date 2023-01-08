// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/utils/math.hpp"
#include "my_math/dcm.hpp"
#include "utest_utils.hpp"

using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::Quat;
using core::utils::RPY;
using core::utils::Vec3;
using my::DCM;

TEST(DCM, DefaultConstructor) {
  const DCM dcm;
  EXPECT_TRUE(ExpectEqMat3(Mat3::Identity(), dcm.Matrix()));
}
