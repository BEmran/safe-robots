#ifndef CORE_MATH_QUATERNION_HPP_
#define CORE_MATH_QUATERNION_HPP_

// #include <algorithm>  // clamp
// #include <iostream>   // cout
#include <cmath>  // acos

#include "core/utils/math.hpp"

using core::utils::Quat;
using core::utils::Vec3;

class Quaternion {
 public:
  Quaternion() : Quaternion(1.f, Vec3::Zero()) {
  }

  Quaternion(const Quat q) : Quaternion(q.w(), q.vec()) {
  }

  Quaternion(const float w, const float x, const float y, const float z)
    : Quaternion{w, Vec3(x, y, z)} {
  }

  Quaternion(const float scalar, const Vec3 vec) : scalar_{scalar}, vec_{vec} {
  }

  void Normalize() {
    const Quaternion quat = Normalized();
    scalar_ = quat.Scalar();
    vec_ = quat.Vec();
  }

  Quaternion Normalized() const {
    constexpr float EPS = 0.001f;
    const float norm = Norm();
    if (norm < EPS) {
      Quaternion(1.f, Vec3::Zero());
    }
    return Quaternion(scalar_ / norm, vec_ / norm);
  }

  void SetIdentity() {
    scalar_ = 1.f;
    vec_ = Vec3::Zero();
  }

  float Norm() const {
    const float square = scalar_ * scalar_ + vec_.dot(vec_);
    return std::sqrt(square);
  }

  float Angle() const {
    return 2 * std::acos(scalar_);
  }

  float Scalar() const {
    return scalar_;
  }

  float& Scalar() {
    return scalar_;
  }

  Vec3 Vec() const {
    return vec_;
  }

  Vec3& Vec() {
    return vec_;
  }

  Quaternion Inverse() {
    return Conjugate();
  }

  Quaternion Conjugate() const {
    return Quaternion(scalar_, -vec_);
  }

  Quaternion Mirror() const {
    return Quaternion(-scalar_, -vec_);
  }

  Vec3 Cross(const Quaternion& rhs) const {
    return Vec().cross(rhs.Vec());
  }

  float Dot(const Quaternion& rhs) const {
    return Vec().dot(rhs.Vec());
  }

  float AngularDistance(const Quaternion& rhs) const {
    const float cos_angle = Dot(rhs) / Norm() / rhs.Norm();
    return std::acos(cos_angle);
  }

  float operator[](const size_t idx) const {
    if (idx == 0) {
      return scalar_;
    }
    return vec_[static_cast<Eigen::Index>(idx + 1)];
  }

  float& operator[](const size_t idx) {
    if (idx == 0) {
      return scalar_;
    }
    return vec_[static_cast<Eigen::Index>(idx + 1)];
  }

  float W() const {
    return scalar_;
  }

  float X() const {
    return vec_.x();
  }

  float Y() const {
    return vec_.x();
  }

  float Z() const {
    return vec_.x();
  }

  float& W() {
    return scalar_;
  }

  float& X() {
    return vec_.x();
  }

  float& Y() {
    return vec_.x();
  }

  float& Z() {
    return vec_.x();
  }

 protected:
 private:
  float scalar_;
  Vec3 vec_;
};

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
                   rhs.Cross(lhs);
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

#endif  // CORE_MATH_QUATERNION_HPP_