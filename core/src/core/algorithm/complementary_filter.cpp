// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/algorithm/complementary_filter.hpp"

#include <cmath>

namespace core::algorithm {
using core::utils::Mat3;

ComplementaryFilter::ComplementaryFilter(
  const ComplementaryFilterConfig& config)
  : config_{config} {
}

std::optional<Quat> ComplementaryFilter::Update(const Vec3& accel,
                                                const Vec3& gyro,
                                                const float dt = -1) {
  if (not(CheckIfVec3IsValid(accel) && CheckIfVec3IsValid(gyro))) {
    return {};
  }
  const float used_dt = dt == -1 ? config_.dt : dt;
  const Quat quat_w = AttitudePropagation(gyro, used_dt);
  const Quat quat_a = OrientationFromAccelerometer(accel);
  quat_ = ComplementaryEstimation(quat_w, quat_a);
  return quat_;
}

std::optional<Quat> ComplementaryFilter::Update(const Vec3& accel,
                                                const Vec3& gyro,
                                                const Vec3& mag,
                                                const float dt = -1) {
  if (not(CheckIfVec3IsValid(accel) && CheckIfVec3IsValid(gyro) &&
          CheckIfVec3IsValid(mag))) {
    return {};
  }
  const float used_dt = dt == -1 ? config_.dt : dt;
  const Quat quat_w = AttitudePropagation(gyro, used_dt);
  const Quat quat_am = OrientationFromAccelerometerMagnetometer(accel, mag);
  quat_ = ComplementaryEstimation(quat_w, quat_am);
  return quat_;
}

bool ComplementaryFilter::CheckIfVec3IsValid(const Vec3& vec) const {
  return vec.norm() > 0;
}

Quat ComplementaryFilter::OrientationFromAccelerometer(
  const Vec3& accel) const {
  // Normalize accelerometer measurements
  const Vec3 a = accel.normalized();
  // Euler Angles from Gravity vector
  const float ex = atan2(a.y(), a.z());
  const float ey = atan2(-a.x(), sqrt(a.y() * a.y() + a.y() * a.z()));
  // Euler to Quaternion
  const float cx2 = cos(ex / 2.0f);
  const float sx2 = sin(ex / 2.0f);
  const float cy2 = cos(ey / 2.0f);
  const float sy2 = sin(ey / 2.0f);
  const Quat q{cx2 * cy2, sx2 * cy2, cx2 * sy2, -sx2 * sy2};
  return q.normalized();
}

Quat ComplementaryFilter::OrientationFromAccelerometerMagnetometer(
  const Vec3& accel, const Vec3& mag) const {
  const Vec3 m = mag.normalized();
  const Vec3 Rz = accel.normalized();
  Vec3 Ry;
  Vec3 Rx;
  if (config_.frame == Frame::NED) {
    Ry = Rz.cross(mag);
    Rx = Ry.cross(Rz);
  } else {
    Rx = m.cross(Rz);
    Ry = Rz.cross(Rx);
  }
  Rx.normalize();
  Ry.normalize();
  Mat3 R;
  R.col(0) << Rx;
  R.col(1) << Ry;
  R.col(2) << Rz;
  R.transposeInPlace();
  const Quat q_out{R};
  return q_out.normalized();
}

Quat ComplementaryFilter::AttitudePropagation(const Vec3& gyro,
                                              const float dt) const {
  const Vec3 w = -0.5 * dt * gyro;
  Eigen::Matrix<core::utils::MATH_TYPE, 4, 4> A;
  A << 1.0f, -w[0], -w[2], -w[2],  //
    w[0], 1.0f, w[2], -w[1],       //
    w[1], -w[2], 1.0f, w[0],       //
    w[2], w[1], -w[0], 1.0f;
  const Quat q_omega{A * quat_.coeffs()};
  return q_omega.normalized();
}

Quat ComplementaryFilter::ComplementaryEstimation(const Quat& quat_w,
                                                  const Quat& quat_am) const {
  static const float sqrt_2 = sqrt(2.0f);
  const float w_gain = 1.0 - config_.gain;
  const Quat tmp = quat_w * quat_am;
  if (tmp.norm() < sqrt_2) {
    const Quat out{quat_w.coeffs() * w_gain - quat_am.coeffs() * config_.gain};
    return out.normalized();

  } else {
    const Quat out{quat_w.coeffs() * w_gain + quat_am.coeffs() * config_.gain};
    return out.normalized();
  }
}

}  // namespace core::algorithm