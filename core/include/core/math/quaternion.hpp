#ifndef CORE_MATH_QUATERNION_HPP_
#define CORE_MATH_QUATERNION_HPP_

#include "core/math/math.hpp"

namespace math {
using core::math::Quat;
using core::math::Vec3;

class Quaternion {
 public:
  Quaternion();

  Quaternion(const Quat q);

  Quaternion(const float w, const float x, const float y, const float z);

  Quaternion(const float scalar, const Vec3 vec);

  void Normalize();

  Quaternion Normalized() const;

  float Norm() const;

  void SetIdentity();

  float Angle() const;

  Quaternion Conjugate() const;

  float Dot(const Quaternion& rhs) const;

  float AngularDistance(const Quaternion& rhs) const;

  float operator[](const size_t idx) const;

  float& operator[](const size_t idx);

  inline float Scalar() const {
    return scalar_;
  }

  inline float& Scalar() {
    return scalar_;
  }

  inline Vec3 Vec() const {
    return vec_;
  }

  inline Vec3& Vec() {
    return vec_;
  }

  inline float W() const {
    return scalar_;
  }

  inline float X() const {
    return vec_.x();
  }

  inline float Y() const {
    return vec_.y();
  }

  inline float Z() const {
    return vec_.z();
  }

  inline float& W() {
    return scalar_;
  }

  inline float& X() {
    return vec_.x();
  }

  inline float& Y() {
    return vec_.y();
  }

  inline float& Z() {
    return vec_.z();
  }

 protected:
 private:
  float scalar_;
  Vec3 vec_;
};

Quaternion QuaternionFromRotation(const float angle, const Vec3 axis);

Quaternion operator+(const Quaternion lhs, const Quaternion rhs);
Quaternion operator-(const Quaternion lhs, const Quaternion rhs);
Quaternion operator*(const Quaternion lhs, const Quaternion rhs);
Quaternion operator*(const Quaternion quat, const Vec3& point);
Quaternion operator*(const Quaternion quat, const float scale);
Quaternion operator/(const Quaternion quat, const float scale);

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
//   if (angular_distance > core::math::PI) {
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
}  // namespace math
#endif  // CORE_MATH_QUATERNION_HPP_