#include "core/math/quaternion.hpp"

#include <cmath>      // acos
#include <stdexcept>  // out_of_range

#include "core/math/math.hpp"
#include "core/utils/logger_macros.hpp"

namespace core::math {
Quaternion::Quaternion() : Quaternion(1.f, Vec3::Zero()) {
}

Quaternion::Quaternion(const Quat q) : Quaternion(q.w(), q.vec()) {
}

Quaternion::Quaternion(const float w, const float x, const float y,
                       const float z)
  : Quaternion{w, Vec3(x, y, z)} {
}

Quaternion::Quaternion(const float scalar, const Vec3 vec)
  : scalar_{scalar}, vec_{vec} {
}

void Quaternion::Normalize() {
  const Quaternion quat = Normalized();
  scalar_ = quat.Scalar();
  vec_ = quat.Vec();
}

Quaternion Quaternion::Normalized() const {
  constexpr float EPS = 0.001f;
  const float norm = Norm();
  if (norm < EPS) {
    return Quaternion(1.f, Vec3::Zero());
  }
  return Quaternion(scalar_ / norm, vec_ / norm);
}

float Quaternion::Norm() const {
  const float dot = scalar_ * scalar_ + vec_.dot(vec_);
  return std::sqrt(dot);
}

void Quaternion::SetIdentity() {
  scalar_ = 1.f;
  vec_ = Vec3::Zero();
}

float Quaternion::Angle() const {
  const float ang = 2 * std::atan2(Vec().norm(), W());  // signed angle
  if (ang >= 2.f * PI) {
    return ang - 2.f * PI;
  } else if (ang <= -2.f * PI) {
    return ang + 2.f * PI;
  } else {
    return ang;
  }
  // return 2 * std::acos(std::abs(scalar_));  // always positive
}

Quaternion Quaternion::Conjugate() const {
  return Quaternion(scalar_, -vec_);
}

float Quaternion::Dot(const Quaternion& rhs) const {
  return scalar_ * rhs.Scalar() + Vec().dot(rhs.Vec());
}

float Quaternion::AngularDistance(const Quaternion& rhs) const {
  const Quaternion a = this->Normalized();
  const Quaternion b = rhs.Normalized();
  // const float dot =
  //   a.W() * b.W() + a.X() * b.X() + a.Y() * b.Y() + a.Z() * b.Z();
  // return std::acos(2 * dot - 1);

  const Quaternion relative_rotation = a.Conjugate() * b;
  return relative_rotation.Angle();
}

float Quaternion::operator[](const size_t idx) const {
  if (idx > 4) {
    throw std::out_of_range("expected index values in range of [0, 4)");
  }
  if (idx == 0) {
    return scalar_;
  }
  return vec_[static_cast<Eigen::Index>(idx + 1)];
}

float& Quaternion::operator[](const size_t idx) {
  if (idx > 4) {
    throw std::out_of_range("expected index values in range of [0, 4)");
  }
  if (idx == 0) {
    return scalar_;
  }
  return vec_[static_cast<Eigen::Index>(idx + 1)];
}

Quaternion QuaternionFromRotation(const float angle, const Vec3 axis) {
  const float half_angle = angle / 2.f;
  const float sin_half_angle = std::sin(half_angle);
  return Quaternion(std::cos(half_angle), axis * sin_half_angle);
}

Quaternion operator+(const Quaternion lhs, const Quaternion rhs) {
  const float scalar = lhs.Scalar() + rhs.Scalar();
  const Vec3 vec = lhs.Vec() + rhs.Vec();
  return Quaternion(scalar, vec);
}

Quaternion operator-(const Quaternion lhs, const Quaternion rhs) {
  const float scalar = lhs.Scalar() - rhs.Scalar();
  const Vec3 vec = lhs.Vec() - rhs.Vec();
  return Quaternion(scalar, vec);
}

Quaternion operator*(const Quaternion lhs, const Quaternion rhs) {
  const float scalar = lhs.Scalar() * rhs.Scalar() - lhs.Vec().dot(rhs.Vec());
  const Vec3 vec = lhs.Scalar() * rhs.Vec() +  //
                   rhs.Scalar() * lhs.Vec() +  //
                   lhs.Vec().cross(rhs.Vec());
  return Quaternion(scalar, vec);
}

Quaternion operator*(const Quaternion quat, const Vec3& point) {
  return quat * Quaternion(0, point) * quat.Conjugate();
}

Quaternion operator*(const Quaternion quat, const float scale) {
  return Quaternion(quat.Scalar() * scale, quat.Vec() * scale);
}

Quaternion operator*(const float scale, const Quaternion quat) {
  return quat * scale;
}

Quaternion operator/(const Quaternion quat, const float scale) {
  return Quaternion(quat.Scalar() / scale, quat.Vec() / scale);
}

Quaternion LinearInterpolation(const Quaternion& qf, const Quaternion& qs,
                               const float interpolation_point) {
  return qf + interpolation_point * (qs - qf);
}

Quaternion SphericalLinearInterpolation(const Quaternion& qf,
                                        const Quaternion& qs,
                                        const float interpolation_points) {
  // Angle between vectors
  float angular_distance = qf.AngularDistance(qs);
  const float sin_theta = std::sin(angular_distance);
  const float theta_threshold = angular_distance * interpolation_points;
  const float s0 = std::sin(angular_distance - theta_threshold) / sin_theta;
  const float s1 = std::sin(theta_threshold) / sin_theta;
  return qf * s0 + qs * s1;
}

std::vector<Quaternion>
QuaternionInterpolation(const Quaternion& qf, const Quaternion& qs,
                        const std::vector<float>& interpolation_points) {
  constexpr float angular_distance_threshold = 0.05f;
  const Quaternion q0 = qf.Normalized();
  Quaternion q1 = qs.Normalized();
  float angular_distance = q0.AngularDistance(q1);

  // Ensure SLERP takes the shortest path
  if (angular_distance > PI) {
    SYS_LOG_WARN("Quaternion Interpolation: ") << "flip Second endpoint "
                                                  "quaternion to takes the "
                                                  "shortest path";
    q1 = Quaternion(-qs.Scalar(), -qs.Vec()).Normalized();
    // angular_distance *= -1;
    angular_distance = q0.AngularDistance(q1);
  }

  std::vector<Quaternion> quat(interpolation_points.size());
  for (size_t i = 0; i < interpolation_points.size(); i++) {
    if (angular_distance < angular_distance_threshold) {
      quat[i] = LinearInterpolation(q0, q1, interpolation_points[i]);
    } else {
      quat[i] = SphericalLinearInterpolation(q0, q1, interpolation_points[i]);
    }
  }
  return quat;
}

}  // namespace core::math