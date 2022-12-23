#include "core/algorithm/madgwick.hpp"

#include <cmath>

namespace core::algorithm {
using core::utils::Mat3;

Madgwick::Madgwick(const MadgwickConfig config) : config_{config} {
}

std::optional<Quat> Madgwick::Update(const Vec3& accel, const Vec3& gyro,
                                     const float dt = -1) {
  if (not(CheckIfVec3IsValid(accel) && CheckIfVec3IsValid(gyro))) {
    return {};
  }
  const float used_dt = dt == -1 ? config_.dt : dt;
  const Vec3 a = accel.normalized();

  Vec3 f;
  f.x() = 2 * (quat_.x() * quat_.z() - quat_.w() * quat_.y()) - a.x();
  f.y() = 2 * (quat_.w() * quat_.x() + quat_.y() * quat_.z()) - a.y();
  f.z() = 2 * (0.5 - quat_.x() * quat_.x() - quat_.x() * quat_.x()) - a.z();

  if (f.norm() < 0) {
    return {};
  }

  Eigen::Matrix<core::utils::MATH_TYPE, 3, 4> J;
  J << -2 * quat_.y(), 2 * quat_.z(), -2 * quat_.w(), 2 * quat_.x(),  //
    2 * quat_.x(), 2 * quat_.w(), 2 * quat_.z(), 2 * quat_.y(),       //
    0.f, -4 * quat_.x(), -4 * quat_.y(), 0.f;                         //

  Quat gradient{J.transpose() * f};
  gradient.normalize();
  const Quat q_omega{0, gyro.x(), gyro.y(), gyro.z()};
  Quat q_dot{0.5 * quat_.coeffs() * q_omega.coeffs()};
  q_dot.coeffs() = q_dot.coeffs() - config_.gain * gradient.coeffs();
  Quat quat_new{quat_.coeffs() + q_dot.coeffs() * used_dt};
  quat_new.normalize();
  quat_ = quat_new;
  return quat_;
}

std::optional<Quat> Madgwick::Update(const Vec3& accel, const Vec3& gyro,
                                     const Vec3& mag, const float dt = -1) {
  if (not(CheckIfVec3IsValid(accel) && CheckIfVec3IsValid(gyro) &&
          CheckIfVec3IsValid(mag))) {
    return {};
  }
  const float used_dt = dt == -1 ? config_.dt : dt;

  const Vec3 a = accel.normalized();
  const Vec3 m = mag.normalized();
  const Quat q_mag{0, m.x(), m.y(), m.z()};

  // Rotate normalized magnetometer measurements
  const Quat h = quat_ * q_mag * quat_.conjugate();
  const float bx = norm([ h[1], h[2] ]);
  const float bz = h.z();

  Eigen::Matrix<core::utils::MATH_TYPE, 6, 1> f;
  f << 2 * (quat_.x() * quat_.z() - quat_.w() * quat_.y()) - a.x(),     //
    2 * (quat_.w() * quat_.x() + quat_.y() * quat_.z()) - a.y(),        //
    2 * (0.5 - quat_.x() * quat_.x() - quat_.y() * quat_.y()) - a.z(),  //
    2 * bx * (0.5 - quat_.y() * quat_.y() - quat_.z() * quat_.z()) +
      2 * bz * (quat_.x() * quat_.z() - quat_.w() * quat_.y()) - m.x(),  //
    2 * bx * (quat_.x() * quat_.y() - quat_.w() * quat_.z()) +
      2 * bz * (quat_.w() * quat_.x() + quat_.y() * quat_.z()) - m.y(),  //
    2 * bx * (quat_.w() * quat_.y() + quat_.x() * quat_.z()) +
      2 * bz * (0.5 - quat_.x() * quat_.x() - quat_.y() * quat_.y()) - m.z();

  Eigen::Matrix<core::utils::MATH_TYPE, 6, 4> J;
  J << -2 * quat_.y(), 2 * quat_.z(), -2 * quat_.w(), 2 * quat_.x(),  //
    2 * quat_.x(), 2 * quat_.w(), 2 * quat_.z(), 2 * quat_.y(),       //
    0.0, -4.0 * quat_.x(), -4.0 * quat_.y(), 0.0,                     //

    -2 * bz * quat_.y(), 2 * bz * quat_.z(),
    -4 * bx * quat_.y() - 2 * bz * quat_.w(),
    -4 * bx * quat_.z() + 2 * bz * quat_.x(),

    -2 * bx * quat_.z() + 2 * bz * quat_.x(),
    2 * bx * quat_.y() + 2 * bz * quat_.w(),
    2 * bx * quat_.x() + 2 * bz * quat_.z(),
    -2 * bx * quat_.w() + 2 * bz * quat_.y(),

    2 * bx * quat_.y(), 2 * bx * quat_.z() - 4 * bz * quat_.x(),
    2 * bx * quat_.w() - 4 * bz * quat_.y(), 2 * bx * quat_.x();

  Quat gradient{J.transpose() * f};
  gradient.normalize();
  const Quat q_omega{0, gyro.x(), gyro.y(), gyro.z()};
  Quat q_dot{0.5 * quat_.coeffs() * q_omega.coeffs()};
  q_dot.coeffs() = q_dot.coeffs() - config_.gain * gradient.coeffs();
  Quat quat_new{quat_.coeffs() + q_dot.coeffs() * used_dt};
  quat_new.normalize();
  quat_ = quat_new;
  return quat_;
}

bool Madgwick::CheckIfVec3IsValid(const Vec3& vec) const {
  return vec.norm() > 0;
}
}  // namespace core::algorithm