// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_ALGORITHM_COMPLEMENTARY_FILTER_HPP_
#define CORE_ALGORITHM_COMPLEMENTARY_FILTER_HPP_

#include <optional>

#include "core/utils/data.hpp"
#include "core/utils/math.hpp"

namespace core::algorithm {
using core::utils::Quat;
using core::utils::Vec3;

enum class Frame { NED, ENU };

struct ComplementaryFilterConfig {
  float dt{0.01};
  float gain{0.9};
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
                             const float dt = -1);

  // Attitude Estimation from given measurements and previous orientation.
  std::optional<Quat> Update(const Vec3& accel, const Vec3& gyro,
                             const Vec3& mag, const float dt = -1);

 protected:
  bool CheckIfVec3IsValid(const Vec3& vec) const;

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

}  // namespace core::algorithm
#endif  // CORE_ALGORITHM_COMPLEMENTARY_FILTER_HPP_