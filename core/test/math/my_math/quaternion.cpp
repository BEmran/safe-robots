#include "quaternion.hpp"

#include <cmath>  // acos

namespace my {
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
    Quaternion(1.f, Vec3::Zero());
  }
  return Quaternion(scalar_ / norm, vec_ / norm);
}

float Quaternion::Norm() const {
  const float square = scalar_ * scalar_ + vec_.dot(vec_);
  return std::sqrt(square);
}

void Quaternion::SetIdentity() {
  scalar_ = 1.f;
  vec_ = Vec3::Zero();
}

float Quaternion::Angle() const {
  // return 2 * std::atan2(Vec().norm(), W()); // signed angle
  return 2 * std::acos(std::abs(scalar_));  // always positive
}

Quaternion Quaternion::Conjugate() const {
  return Quaternion(scalar_, -vec_);
}

Vec3 Quaternion::Cross(const Quaternion& rhs) const {
  return Vec().cross(rhs.Vec());
}

float Quaternion::Dot(const Quaternion& rhs) const {
  return Vec().dot(rhs.Vec());
}

float Quaternion::Dot2(const Quaternion& rhs) const {
  return scalar_ * rhs.Scalar() + Vec().dot(rhs.Vec());
}

float Quaternion::AngularDistance(const Quaternion& rhs) const {
  const Quaternion a = this->Normalized();
  const Quaternion b = rhs.Normalized();
  // const float dot =
  //   a.W() * b.W() + a.X() * b.X() + a.Y() * b.Y() + a.Z() * b.Z();
  // return std::acos(2 * dot - 1);

  const Quaternion relative_rotation = a.Conjugate() * b;
  // return 2 * std::atan2(relative_rotation.Vec().norm(), relative_rotation.W());
  return relative_rotation.Angle();
}

float Quaternion::operator[](const size_t idx) const {
  if (idx == 0) {
    return scalar_;
  }
  return vec_[static_cast<Eigen::Index>(idx + 1)];
}

float& Quaternion::operator[](const size_t idx) {
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
  const float scalar = lhs.Scalar() * rhs.Scalar() - rhs.Dot(lhs);
  const Vec3 vec = lhs.Scalar() * rhs.Vec() +  //
                   rhs.Scalar() * lhs.Vec() +  //
                   lhs.Cross(rhs);
  return Quaternion(scalar, vec);
}

Quaternion operator*(const Quaternion quat, const Vec3& point) {
  return quat * Quaternion(0, point) * quat.Conjugate();
}

Quaternion operator*(const Quaternion quat, const float scale) {
  const float scalar = quat.Scalar() * scale;
  const Vec3 vec = quat.Vec() * scale;
  return Quaternion(scalar, vec);
}

Quaternion operator/(const Quaternion quat, const float scale) {
  const float scalar = quat.Scalar() / scale;
  const Vec3 vec = quat.Vec() / scale;
  return Quaternion(scalar, vec);
}

// /**
//  * @brief Interpolate linearly (LERP)
//  *
//  * @param qs First endpoint quaternion
//  * @param q2 Second endpoint quaternion
//  * @param interpolation_points an interpolation point between 0 and 1
//  * @return Quat
//  */
// Quat LinearInterpolation(const Quat& qf, const Quat& qs,
//                          const float interpolation_points) {
//   return qf + interpolation_points * (qs - qf);
// }

// /**
//  * @brief Spherical Linear Interpolation between two quaternions.
//  *
//  * @param qs First endpoint quaternion
//  * @param q2 Second endpoint quaternion
//  * @param interpolation_points an interpolation point between 0 and 1
//  * @return Quat
//  */
// Quat SphericalLinearInterpolation(const Quat& qf, const Quat& qs,
//                                   const float interpolation_points) {
//   // Angle between vectors
//   float angular_distance = qf.angularDistance(qs);
//   const float sin_theta = std::sin(angular_distance);
//   const float theta_threshold = angular_distance * interpolation_points;
//   const float s0 = std::sin(angular_distance - theta_threshold) / sin_theta;
//   const float s1 = std::sin(theta_threshold) / sin_theta;
//   return s0 * qf + s1 * qs;
// }
// /**
//  * @brief Spherical Linear Interpolation between two quaternions. Return a
//  valid
//  * quaternion rotation at a specified distance along the minor arc of a great
//  * circle passing through any two existing quaternion endpoints lying on the
//  * unit radius hypersphere. Based on the method detailed in [Wiki_SLERP]_
//  *
//  * @param qs First endpoint quaternion
//  * @param q2 Second endpoint quaternion
//  * @param interpolation_points a vector of points from 0 to 1
//  * @return Quat quaternion representing the interpolated rotation
//  */
// std::vector<Quat>
// QuaternionInterpolation(const Quat& qf, const Quat& qs,
//                         const std::vector<float>& interpolation_points) {
//   constexpr float angular_distance_threshold = 0.05;
//   Quat q0 = qf.normalized();
//   Quat q1 = qs.normalized();
//   float angular_distance = qf.angularDistance(qs);

//   // Ensure SLERP takes the shortest path
//   if (angular_distance > core::utils::PI) {
//     std::cout << "flip Second endpoint quaternion to takes the shortest path"
//               << std::endl;
//     angular_distance *= -1;
//     q1 = Quat(-qs.w(), -qs.x(), -qs.y(), -qs.z()).normalized();
//   }

//   std::vector<Quat> quat(interpolation_points.size());
//   for (size_t i = 0; i < interpolation_points.size(); i++) {
//     if (angular_distance < angular_distance_threshold) {
//       quat[i] = LinearInterpolation(q0, q1, interpolation_points[i]);
//     } else {
//       quat[i] = SphericalLinearInterpolation(q0, q1,
//       interpolation_points[i]);
//     }
//   }
//   return quat;
// }

}  // namespace my