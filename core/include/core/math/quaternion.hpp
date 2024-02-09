#ifndef CORE_MATH_QUATERNION_HPP_
#define CORE_MATH_QUATERNION_HPP_

#include "core/math/math.hpp"

namespace core::math {
using core::math::Quat;
using core::math::Vec3;

class Quaternion {
 public:
  Quaternion();

  Quaternion(const Quat& q);

  Quaternion(float w, float x, float y, float z);

  Quaternion(float scalar, Vec3 vec);

  void Normalize();

  Quaternion Normalized() const;

  float Norm() const;

  void SetIdentity();

  float Angle() const;

  Quaternion Conjugate() const;

  float Dot(const Quaternion& rhs) const;

  float AngularDistance(const Quaternion& rhs) const;

  float operator[](size_t idx) const;

  float& operator[](size_t idx);

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

Quaternion QuaternionFromRotation(float angle, const Vec3& axis);

Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator-(const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator*(const Quaternion& quat, const Vec3& point);
Quaternion operator*(const Quaternion& quat, float scale);
Quaternion operator*(float scale, const Quaternion& quat);
Quaternion operator/(const Quaternion& quat, float scale);

/**
 * @brief Interpolate linearly (LERP)
 *
 * @param qs First endpoint quaternion
 * @param q2 Second endpoint quaternion
 * @param interpolation_point an interpolation point between 0 and 1
 * @return Quat
 */
Quaternion LinearInterpolation(const Quaternion& qf, const Quaternion& qs,
                               float interpolation_point);

/**
 * @brief Spherical Linear Interpolation between two quaternions. Return a valid
 * quaternion rotation at a specified distance along the minor arc of a great
 * circle passing through any two existing quaternion endpoints lying on the
 * unit radius hypersphere. Based on the method detailed in [Wiki_SLERP]_
 *
 * @param qs First endpoint quaternion
 * @param q2 Second endpoint quaternion
 * @param interpolation_points an interpolation point between 0 and 1
 * @return Quaternion
 */
Quaternion SphericalLinearInterpolation(const Quaternion& qf,
                                        const Quaternion& qs,
                                        float interpolation_points);

/**
 * @brief Quaternion Interpolation between two quaternions.
 *
 * @param qs First endpoint quaternion
 * @param q2 Second endpoint quaternion
 * @param interpolation_points a vector of points from 0 to 1
 * @return Quat quaternion representing the interpolated rotation
 */
std::vector<Quaternion>
QuaternionInterpolation(const Quaternion& qf, const Quaternion& qs,
                        const std::vector<float>& interpolation_points);

}  // namespace core::math
#endif  // CORE_MATH_QUATERNION_HPP_