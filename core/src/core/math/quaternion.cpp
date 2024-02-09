#include "core/math/quaternion.hpp"

#include <cmath>      // acos
#include <stdexcept>  // out_of_range

#include "core/math/math.hpp"
#include "core/utils/logger_macros.hpp"

namespace core::math {
Quaternion::Quaternion() : Quaternion(1.F, Vec3::Zero()) {
}

Quaternion::Quaternion(const Quat& q) : Quaternion(q.w(), q.vec()) {
}

Quaternion::Quaternion(float w, float x, float y, float z)
  : Quaternion{w, Vec3(x, y, z)} {
}

Quaternion::Quaternion(float scalar, Vec3 vec)
  : scalar_{scalar}, vec_{std::move(vec)} {
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
    return Quaternion(1.F, Vec3::Zero());
  }
  return Quaternion(scalar_ / norm, vec_ / norm);
}

float Quaternion::Norm() const {
  const float dot = scalar_ * scalar_ + vec_.dot(vec_);
  return std::sqrt(dot);
}

void Quaternion::SetIdentity() {
  scalar_ = 1.F;
  vec_ = Vec3::Zero();
}

float Quaternion::Angle() const {
  const float ang = 2 * std::atan2(Vec().norm(), W());  // signed angle [0,
  // 2*PI]
  // const float ang = 2 * std::acos(W());  // always positive [0, 2*PI)
  if (ang >= 2 * PI) {
    return ang - 2 * PI;
  }
  return ang;
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
  const Quaternion relative_rotation = a * b.Conjugate();
  const float ang = relative_rotation.Angle();
  if (ang >= PI) {
    return ang - PI;
  } else {
    return ang;
  }
  // return relative_rotation.Angle();
}

// float Quaternion::ShortAngularDistance2(const Quaternion& rhs) const {
//   const Quaternion q0 = this->Normalized();
//   Quaternion q1 = rhs.Normalized();
//   float ang_dist = q0.AngularDistance(q1);
//   // Ensure SLERP takes the shortest path
//   if (ang_dist >= PI) {
//     q1 = Quaternion(-q1.Scalar(), -q1.Vec());
//     ang_dist = q0.AngularDistance(q1);
//   }
// }
float Quaternion::operator[](size_t idx) const {
  if (idx > 4) {
    throw std::out_of_range("expected index values in range of [0, 4)");
  }
  if (idx == 0) {
    return scalar_;
  }
  return vec_[static_cast<Eigen::Index>(idx + 1)];
}

float& Quaternion::operator[](size_t idx) {
  if (idx > 4) {
    throw std::out_of_range("expected index values in range of [0, 4)");
  }
  if (idx == 0) {
    return scalar_;
  }
  return vec_[static_cast<Eigen::Index>(idx + 1)];
}

Quaternion QuaternionFromRotation(float angle, const Vec3& axis) {
  const float half_angle = angle / 2.F;
  const float sin_half_angle = std::sin(half_angle);
  return Quaternion(std::cos(half_angle), axis * sin_half_angle);
}

Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs) {
  const float scalar = lhs.Scalar() + rhs.Scalar();
  const Vec3 vec = lhs.Vec() + rhs.Vec();
  return Quaternion(scalar, vec);
}

Quaternion operator-(const Quaternion& lhs, const Quaternion& rhs) {
  const float scalar = lhs.Scalar() - rhs.Scalar();
  const Vec3 vec = lhs.Vec() - rhs.Vec();
  return Quaternion(scalar, vec);
}

Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs) {
  const float scalar = lhs.Scalar() * rhs.Scalar() - lhs.Vec().dot(rhs.Vec());
  const Vec3 vec = lhs.Scalar() * rhs.Vec() +  //
                   rhs.Scalar() * lhs.Vec() +  //
                   lhs.Vec().cross(rhs.Vec());
  return Quaternion(scalar, vec);
}

Quaternion operator*(const Quaternion& quat, const Vec3& point) {
  return quat * Quaternion(0, point) * quat.Conjugate();
}

Quaternion operator*(const Quaternion& quat, const float scale) {
  return Quaternion(quat.Scalar() * scale, quat.Vec() * scale);
}

Quaternion operator*(float scale, const Quaternion& quat) {
  return quat * scale;
}

Quaternion operator/(const Quaternion& quat, float scale) {
  return Quaternion(quat.Scalar() / scale, quat.Vec() / scale);
}

Quaternion LinearInterpolation(const Quaternion& qf, const Quaternion& qs,
                               float interpolation_point) {
  return qf + interpolation_point * (qs - qf);
}

Quaternion SphericalLinearInterpolation(const Quaternion& qf,
                                        const Quaternion& qs,
                                        float interpolation_points) {
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
    const auto a = angular_distance;
    q1 = Quaternion(-qs.Scalar(), -qs.Vec()).Normalized();
    angular_distance = q0.AngularDistance(q1);
    SYS_LOG_WARN("Quaternion Interpolation: ")
      << "flip Second endpoint quaternion to takes the shortest path: " << a
      << " -> " << angular_distance;
  }

  std::vector<Quaternion> quat(interpolation_points.size());
  for (size_t i = 0; i < interpolation_points.size(); i++) {
    if (angular_distance < angular_distance_threshold) {
      quat[i] =
        LinearInterpolation(q0, q1, interpolation_points[i]).Normalized();
    } else {
      quat[i] = SphericalLinearInterpolation(q0, q1, interpolation_points[i])
                  .Normalized();
    }
  }
  return quat;
}

}  // namespace core::math