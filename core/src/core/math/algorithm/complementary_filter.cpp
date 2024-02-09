// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/math/algorithm/complementary_filter.hpp"

#include <algorithm>
#include <cmath>

namespace core::math::algorithm {
namespace {
constexpr float MIN_GAIN = 0.F;
constexpr float MAX_GAIN = 1.F;
}  // namespace

ComplementaryFilter::ComplementaryFilter(
  const ComplementaryFilterConfig& config)
  : config_{config}, quat_{Quat::Identity()} {
}

std::optional<Quat> ComplementaryFilter::Update(const Vec3& accel,
                                                const Vec3& gyro, float dt) {
  if (IsNearZero(accel) || IsNearZero(gyro)) {
    return {};
  }
  const float used_dt = DecideWhichDtToUse(dt);
  const Quat quat_w = AttitudePropagation(gyro, used_dt);
  const Quat quat_a = OrientationFromAccelerometer(accel);
  quat_ = ComplementaryEstimation(quat_w, quat_a);
  return quat_;
}

std::optional<Quat> ComplementaryFilter::Update(const Vec3& accel,
                                                const Vec3& gyro,
                                                const Vec3& mag, float dt) {
  if (IsNearZero(accel) || IsNearZero(gyro) || IsNearZero(mag)) {
    return {};
  }
  const float used_dt = DecideWhichDtToUse(dt);
  const Quat quat_w = AttitudePropagation(gyro, used_dt);
  const Quat quat_am = OrientationFromAccelerometerMagnetometer(accel, mag);
  quat_ = ComplementaryEstimation(quat_w, quat_am);
  return quat_;
}

bool ComplementaryFilter::IsNearZero(const Vec3& vec) const {
  constexpr float eps = 0.001f;
  return vec.norm() < eps;
}

float ComplementaryFilter::DecideWhichDtToUse(float dt) const {
  if (dt > 0.F) {
    return dt;
  }
  return config_.dt;
}

Quat ComplementaryFilter::AttitudePropagation(const Vec3& gyro,
                                              float dt) const {
  const Vec3 w = -0.5f * dt * gyro;
  Eigen::Matrix<core::math::MATH_TYPE, 4, 4> A;
  A << 1.0f, -w[0], -w[1], -w[2],  //
    w[0], 1.0f, w[2], -w[1],       //
    w[1], -w[2], 1.0f, w[0],       //
    w[2], w[1], -w[0], 1.0f;
  Eigen::Vector4f q_coff;
  q_coff << quat_.w(), quat_.x(), quat_.y(), quat_.z();
  Eigen::Vector4f mul;
  mul << A * q_coff;
  const Quat q_omega{mul[0], mul[1], mul[2], mul[3]};
  return q_omega.normalized();

  // Quat q_omega{
  //   1.0f * quat_[0] - w[0] * quat_[1] - w[1] * quat_[2] - w[2] * quat_[3], //
  //   w[0] * quat_[0] + 1.0f * quat_[1] + w[2] * quat_[2] - w[1] * quat_[3], //
  //   w[1] * quat_[0] - w[2] * quat_[1] + 1.0f * quat_[2] + w[0] * quat_[3], //
  //   w[2] * quat_[0] + w[1] * quat_[1] - w[0] * quat_[2] + 1.0f * quat_[3]};
}

Quat ComplementaryFilter::OrientationFromAccelerometer(
  const Vec3& accel) const {
  // Normalize accelerometer measurements
  const Vec3 a = accel.normalized();
  // Euler Angles from Gravity vector
  const float ex = std::atan2(a.y(), a.z());
  const float ey = std::atan2(-a.x(), std::sqrt(a.y() * a.y() + a.z() * a.z()));
  // Euler to Quaternion
  const float cx2 = std::cos(ex / 2.0f);
  const float sx2 = std::sin(ex / 2.0f);
  const float cy2 = std::cos(ey / 2.0f);
  const float sy2 = std::sin(ey / 2.0f);
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
  core::math::Mat3 R;
  R.col(0) << Rx;
  R.col(1) << Ry;
  R.col(2) << Rz;
  R.transposeInPlace();
  const Quat q_out{R};
  return q_out.normalized();
}

Quat ComplementaryFilter::ComplementaryEstimation(const Quat& quat_w,
                                                  const Quat& quat_am) const {
  // static const float sqrt_2 = sqrt(2.0f);
  // const float w_gain = 1.0 - config_.gain;
  // const Quat tmp = quat_w * quat_am;
  // if (tmp.norm() < sqrt_2) {
  //   const Quat out{quat_w.coeffs() * w_gain - quat_am.coeffs() *
  //   config_.gain}; return out.normalized();

  // } else {
  //   const Quat out{quat_w.coeffs() * w_gain + quat_am.coeffs() *
  //   config_.gain}; return out.normalized();
  // }
  // const float w_gain = 1.F - config_.gain;
  return quat_w.slerp(config_.gain, quat_am).normalized();
}

Quat ComplementaryFilter::GetQuat() const {
  return quat_;
}

float ComplementaryFilter::GetGain() const {
  return config_.gain;
}

Quat ComplementaryFilter::Reset(const Quat& quat) {
  return quat_ = quat;
}

float ComplementaryFilter::ResetGain(float gain) {
  config_.gain = std::clamp(gain, MIN_GAIN, MAX_GAIN);
  return config_.gain;
}
}  // namespace core::math::algorithm