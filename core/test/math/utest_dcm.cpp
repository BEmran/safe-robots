// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/math/math.hpp"
#include "core/math/dcm.hpp"
#include "utest_utils.hpp"

using core::math::Mat3;
using core::math::MATH_TYPE;
using core::math::Quat;
using core::math::RPY;
using core::math::Vec3;
using math::DCM;

TEST(DCM, DefaultConstructor) {
  const DCM dcm;
  EXPECT_TRUE(ExpectEqMat3(Mat3::Identity(), dcm.Matrix()));
}
