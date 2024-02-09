// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_MATH_ALGORITHM_COMPLEMENTARY_FILTER_HPP_
#define CORE_MATH_ALGORITHM_COMPLEMENTARY_FILTER_HPP_

#include <optional>

#include "core/math/math.hpp"
#include "core/utils/data.hpp"

namespace core::math::algorithm {
using core::math::Quat;
using core::math::Vec3;

constexpr float DEFAULT_GAIN = 0.9F;

enum class Frame { NED, ENU };

struct ComplementaryFilterConfig {
  /// @brief sampling time
  float dt{0.01F};
  /// @brief gain ratio to blend Gyro sensor sensor with other sensor values.
  /// the value is expected to be in range of [0, 1]. Where 0 means use gyro
  /// sensor only while 1 means use other sensor values only.
  float gain{DEFAULT_GAIN};
  /// @brief sensor frame
  Frame frame{Frame::NED};
};

/**
 * @brief Attitude quaternion obtained with gyroscope and
 * accelerometer-magnetometer measurements, via complementary filter.
 *
 */
class ComplementaryFilter {
 public:
  ComplementaryFilter(const ComplementaryFilterConfig& config);

  // Attitude Estimation from given measurements and previous orientation.
  std::optional<Quat> Update(const Vec3& accel, const Vec3& gyro,
                             float dt = -1.F);

  // Attitude Estimation from given measurements and previous orientation.
  std::optional<Quat> Update(const Vec3& accel, const Vec3& gyro,
                             const Vec3& mag, float dt = -1.F);
  Quat GetQuat() const;
  float GetGain() const;

  Quat Reset(const Quat& quat = Quat::Identity());
  float ResetGain(float gain = DEFAULT_GAIN);

 protected:
  bool IsNearZero(const Vec3& vec) const;

  float DecideWhichDtToUse(float dt) const;

  // Quaternion from roll-pitch-yaw angles
  Quat OrientationFromAccelerometer(const Vec3& accel) const;

  // Orientation from accelerometer and magnetometer readings
  Quat OrientationFromAccelerometerMagnetometer(const Vec3& accel,
                                                const Vec3& mag) const;
  // Attitude propagation of the orientation.
  Quat AttitudePropagation(const Vec3& gyro, const float dt) const;

  // Combine estimation from different source
  Quat ComplementaryEstimation(const Quat& quat_w, const Quat& quat_am) const;

 private:
  ComplementaryFilterConfig config_;
  Quat quat_{Quat().Identity()};
};

}  // namespace core::math::algorithm
#endif  // CORE_ALGORITHM_COMPLEMENTARY_FILTER_HPP_