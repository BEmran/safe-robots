// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../math_utils.hpp"
#include "core/math/algorithm/complementary_filter.hpp"

using core::math::Quat;
using core::math::Vec3;
using core::math::algorithm::ComplementaryFilter;
using core::math::algorithm::ComplementaryFilterConfig;

bool IsParallel(const Vec3& a, const Vec3& b) {
  constexpr float precision = 0.0001f;
  const Vec3 cross{a.cross(b)};
  return cross.isZero(precision);
}

class ComplementaryFilterMock : public ComplementaryFilter {
 public:
  ComplementaryFilterMock(
    const ComplementaryFilterConfig config = ComplementaryFilterConfig())
    : ComplementaryFilter{config} {
  }

  bool RunIsNearZero(const Vec3& vec) const {
    return IsNearZero(vec);
  }

  float RunDecideWhichDtToUse(const float dt) const {
    return DecideWhichDtToUse(dt);
  }

  Quat RunOrientationFromAccelerometer(const Vec3& accel) const {
    return OrientationFromAccelerometer(accel);
  }

  Quat RunOrientationFromAccelerometerMagnetometer(const Vec3& accel,
                                                   const Vec3& mag) const {
    return OrientationFromAccelerometerMagnetometer(accel, mag);
  }

  Quat RunAttitudePropagation(const Vec3& gyro, const float dt) const {
    return AttitudePropagation(gyro, dt);
  }

  Quat RunComplementaryEstimation(const Quat& quat_w,
                                  const Quat& quat_am) const {
    return ComplementaryEstimation(quat_w, quat_am);
  }

 private:
};

TEST(ComplementaryFilterMock, Constructor) {
  // when construct with default config
  const ComplementaryFilterConfig config;
  // then construct passes with no throw
  EXPECT_NO_THROW(ComplementaryFilter{config});
}

TEST(ComplementaryFilterMock, ConstructedWithIdentityQuat) {
  // when construct with default config
  const ComplementaryFilterMock cfm;
  // then the internal quaternion is set to identity
  EXPECT_TRUE(ExpectEqQuaternion(Quat::Identity(), cfm.GetQuat()));
}

TEST(ComplementaryFilterMock, ConstructedWithDefaultGain) {
  // when construct with default config
  ComplementaryFilterMock cfm;
  // then the internal gain is set to that value
  EXPECT_FLOAT_EQ(core::math::algorithm::DEFAULT_GAIN, cfm.GetGain());
}

TEST(ComplementaryFilterMock, RestGainToSomeValue) {
  // when construct with default config
  ComplementaryFilterMock cfm;
  // if ResetGain function is called with some value
  const float expect{0.5f};
  cfm.ResetGain(expect);
  // then the internal gain is set to that value
  EXPECT_FLOAT_EQ(expect, cfm.GetGain());
}

TEST(ComplementaryFilterMock, RestGainToDefault) {
  // when construct with default config
  ComplementaryFilterMock cfm;
  // if ResetGain function is called with no parameter
  cfm.ResetGain();
  // then the internal gain is set to the default value
  EXPECT_FLOAT_EQ(core::math::algorithm::DEFAULT_GAIN, cfm.GetGain());
}

TEST(ComplementaryFilterMock, RestToSomeValue) {
  // when construct with default config
  ComplementaryFilterMock cfm;
  // if Reset function is called
  const Quat expect = Quat::UnitRandom();
  cfm.Reset(expect);
  // then the internal quaternion is reset to the set value
  EXPECT_TRUE(ExpectEqQuaternion(expect, cfm.GetQuat()));
}

TEST(ComplementaryFilterMock, RestToIdentity) {
  // when construct with default config
  ComplementaryFilterMock cfm;
  // if Reset function is called with empty parameter
  cfm.Reset();
  // then the internal quaternion is reset to Identity
  EXPECT_TRUE(ExpectEqQuaternion(Quat::Identity(), cfm.GetQuat()));
}

TEST(ComplementaryFilterMock, IsNearZeroForZeroVector) {
  // when construct with default config
  const ComplementaryFilterMock cfm;
  // if IsNearZero function is called with a vector of zero
  const bool actual = cfm.RunIsNearZero(Vec3::Zero());
  // then the result is true
  EXPECT_TRUE(actual);
}

TEST(ComplementaryFilterMock, IsNearZeroForNoneZeroVector) {
  // when construct with default config
  const ComplementaryFilterMock cfm;
  // if IsNearZero function is called with a vector of none zero
  const bool actual = cfm.RunIsNearZero(Vec3::Random());
  // then the result is false
  EXPECT_FALSE(actual);
}

TEST(ComplementaryFilterMock, DecideWhichDtToUseIfAValuePassedIsPositive) {
  // when construct with default config
  const ComplementaryFilterMock cfm;
  // if DecideWhichDtToUse function is called with any positive value
  const float expect = 0.1f;
  const float actual = cfm.RunDecideWhichDtToUse(expect);
  // then the result is false
  EXPECT_FLOAT_EQ(expect, actual);
}

TEST(ComplementaryFilterMock, DecideWhichDtToUseIfValuePassedIsNegative) {
  // when construct with default config
  const ComplementaryFilterMock cfm;
  // if DecideWhichDtToUse function is called with any negative value
  const float actual = cfm.RunDecideWhichDtToUse(-0.1f);
  // then the return value is what set in config
  EXPECT_FLOAT_EQ(ComplementaryFilterConfig().dt, actual);
}

TEST(ComplementaryFilterMock, DecideWhichDtToUseIfValuePassedIsZero) {
  // when construct with default config
  const ComplementaryFilterMock cfm;
  // if DecideWhichDtToUse function is called with zero
  const float actual = cfm.RunDecideWhichDtToUse(0.f);
  // then the return value is what set in config
  EXPECT_FLOAT_EQ(ComplementaryFilterConfig().dt, actual);
}

TEST(ComplementaryFilterMock, ComplementaryEstimationWithGainZero) {
  // when construct with gain of 0
  ComplementaryFilterConfig config;
  config.gain = 0.f;
  const ComplementaryFilterMock cfm{config};
  // if ComplementaryEstimation function is called with gain of 0
  const Quat gyro_sensor = Quat::UnitRandom();
  const Quat am_sensor = Quat::UnitRandom();
  const Quat actual = cfm.RunComplementaryEstimation(gyro_sensor, am_sensor);
  // then the return value is the gyro sensor values
  EXPECT_TRUE(ExpectEqQuaternion(gyro_sensor, actual));
}

TEST(ComplementaryFilterMock, ComplementaryEstimationWithGainOne) {
  // when construct with gain of 1
  ComplementaryFilterConfig config;
  config.gain = 1.f;
  const ComplementaryFilterMock cfm{config};
  // if ComplementaryEstimation function is called with gain of 1
  const Quat gyro_sensor = Quat::UnitRandom();
  const Quat am_sensor = Quat::UnitRandom();
  const Quat actual = cfm.RunComplementaryEstimation(gyro_sensor, am_sensor);
  // then the return value is the other sensor values
  EXPECT_TRUE(ExpectEqQuaternion(am_sensor, actual));
}

TEST(ComplementaryFilterMock, ComplementaryEstimationWithDefaultGain) {
  // when construct with default config
  const ComplementaryFilterMock cfm;
  // if ComplementaryEstimation function is called with default gain
  const Quat gyro_sensor = Quat::UnitRandom();
  const Quat am_sensor = Quat::UnitRandom();
  const Quat actual = cfm.RunComplementaryEstimation(gyro_sensor, am_sensor);
  // then the return value is interpolation between the two values
  const Quat expect =
    gyro_sensor.slerp(core::math::algorithm::DEFAULT_GAIN, am_sensor);
  EXPECT_TRUE(ExpectEqQuaternion(expect, actual));
}

TEST(ComplementaryFilterMock, AttitudePropagationWithZeroDt) {
  // when construct with default config
  ComplementaryFilterMock cfm;
  // set Quat to some value
  const Quat expect = Quat::UnitRandom();
  cfm.Reset(expect);
  // if AttitudePropagation function is called with zero dt
  const Quat actual = cfm.RunAttitudePropagation(Vec3::Random(), 0.f);
  // then the return value is the current quat value
  EXPECT_TRUE(ExpectEqQuaternion(expect, actual));
}

TEST(ComplementaryFilterMock, AttitudePropagationWithZeroGyro) {
  // when construct with default config
  ComplementaryFilterMock cfm;
  // set Quat to some value
  const Quat expect = Quat::UnitRandom();
  cfm.Reset(expect);
  // if AttitudePropagation function is called with zero gyro value
  const Quat actual = cfm.RunAttitudePropagation(Vec3::Zero(), 1.f);
  // then the return value is the current quat value
  EXPECT_TRUE(ExpectEqQuaternion(expect, actual));
}

TEST(ComplementaryFilterMock, AttitudePropagationWithIdentityQuat) {
  // when construct with default config
  ComplementaryFilterMock cfm;
  // set Quat to identity value
  cfm.Reset();
  // if AttitudePropagation function is called with some vector and dt
  const Vec3 vec = Vec3::Random();
  const float dt = -2.f;
  const Quat actual = cfm.RunAttitudePropagation(vec, dt);
  // then the return value is a quaternion with an axis in the same direction as
  // axis of the used vector
  EXPECT_TRUE(IsParallel(vec, actual.vec()));
}