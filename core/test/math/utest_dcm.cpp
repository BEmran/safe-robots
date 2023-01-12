// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gtest/gtest.h>

#include "core/math/dcm.hpp"
#include "core/math/math.hpp"
#include "utest_utils.hpp"

using core::math::DCM;
using core::math::Mat3;
using core::math::MATH_TYPE;
using core::math::Quat;
using core::math::RPY;
using core::math::Vec3;

TEST(DCM, DefaultConstructor) {
  const DCM dcm;
  EXPECT_TRUE(ExpectEqMat3(Mat3::Identity(), dcm.Matrix()));
}
