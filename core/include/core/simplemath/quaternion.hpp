#pragma once
#include "core/simplemath/basic.hpp"
#include "core/simplemath/vector3.hpp"
#include "core/simplemath/matrix3x3.hpp"
#include "core/simplemath/utility.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>

enum class InterpolationMethod {
  SLERP,  // Spherical linear quaternion interpolation method. This method is
          // most accurate, but also most computation intense.
  LERP,  // Linear quaternion interpolation method. This method is the quickest,
         // but is also least accurate. The method does not always generate
         // normalized output.
  NLERP,  // Normalized quaternion linear interpolation method.
};

template <typename T>
struct Quaternion : public BasicVector4<T> {
  Quaternion() noexcept : BasicVector4<T>(1, 0, 0, 0) {
  }

  Quaternion(T w, T x, T y, T z) noexcept : BasicVector4<T>(w, x, y, z) {
  }

  Quaternion(const std::array<T, 4>& array) noexcept
    : BasicVector4<T>(array[0], array[1], array[2], array[3]) {
  }

  Quaternion(const BasicVector4<T>& bv) noexcept : BasicVector4<T>(bv) {
  }

  Quaternion(T scale, const Vector3<T>& vec) noexcept
    : BasicVector4<T>(scale, vec.x(), vec.y(), vec.z()) {
  }

  inline static Quaternion
  from_angle_and_axis(T angle_rad, const Vector3<T>& axis) noexcept {
    const T c = std::cos(angle_rad / 2);
    const T s = std::sin(angle_rad / 2);
    return Quaternion<T>(c, axis.x() * s, axis.y() * s, axis.z() * s);
  }

  static Quaternion from_matrix(const Matrix3x3<T>& mat) noexcept {
    const T m00 = mat.at(0, 0);
    const T m11 = mat.at(1, 1);
    const T m22 = mat.at(2, 2);
    const T tr = m00 + m11 + m22;
    Quaternion<T> result;

    if (tr > 0) {
      const T s = sqrt(tr + 1.0) * 2.0;  // S=4*qw
      result.w() = 0.25 * s;
      result.x() = (mat.at(2, 1) - mat.at(1, 2)) / s;
      result.y() = (mat.at(0, 2) - mat.at(2, 0)) / s;
      result.z() = (mat.at(1, 0) - mat.at(0, 1)) / s;
    } else if ((m00 > m11) & (m00 > m22)) {
      const T s = sqrt(1.0 + m00 - m11 - m22) * 2.0;  // S=4*qx
      result.w() = (mat.at(2, 1) - mat.at(12)) / s;
      result.x() = 0.25 * s;
      result.y() = (mat.at(0, 1) + mat.at(10)) / s;
      result.z() = (mat.at(0, 2) + mat.at(20)) / s;
    } else if (m11 > m22) {
      const T s = sqrt(1.0 + m11 - m00 - m22) * 2.0;  // S=4*qy
      result.w() = (mat.at(0, 2) - mat.at(2, 0)) / s;
      result.x() = (mat.at(0, 1) + mat.at(1, 0)) / s;
      result.y() = 0.25 * S;
      result.z() = (mat.at(1, 2) + mat.at(2, 1)) / s;
    } else {
      const T s = sqrt(1.0 + m22 - m00 - m11) * 2.0;  // S=4*qz
      result.w() = (mat.at(1, 0) - mat.at(0, 1)) / s;
      result.x() = (mat.at(0, 2) + mat.at(2, 0)) / s;
      result.y() = (mat.at(1, 2) + mat.at(2, 1)) / s;
      result.z() = 0.25 * s;
    }
    return result;
  }

  // static Quaternion from_tb(const Vector3<T>& tb) {
  //   const T tb_2[3];
  //   tb_2[0] = tb.x() / 2.0;
  //   tb_2[1] = tb.y() / 2.0;
  //   tb_2[2] = tb.z() / 2.0;
  //   const T cos_x2 = std::cos(tb_2[0]);
  //   const T sin_x2 = std::sin(tb_2[0]);
  //   const T cos_y2 = std::cos(tb_2[1]);
  //   const T sin_y2 = std::sin(tb_2[1]);
  //   const T cos_z2 = std::cos(tb_2[2]);
  //   const T sin_z2 = std::sin(tb_2[2]);
  //   Quaternion result;
  //   result.w() = cos_x2 * cos_y2 * cos_z2 + sin_x2 * sin_y2 * sin_z2;
  //   result.x() = sin_x2 * cos_y2 * cos_z2 - cos_x2 * sin_y2 * sin_z2;
  //   result.y() = cos_x2 * sin_y2 * cos_z2 + sin_x2 * cos_y2 * sin_z2;
  //   result.z() = cos_x2 * cos_y2 * sin_z2 - sin_x2 * sin_y2 * cos_z2;
  //   return result.normalized();
  // }

  inline static Quaternion random(T vmin, T vmax) {
    return Quaternion<T>(generate_randoms<T, 4>(vmin, vmax));
  }

  inline T scalar() const noexcept {
    return this->w();
  }

  inline Vector3<T> vector() const noexcept {
    return Vector3<T>(this->x(), this->y(), this->z());
  }

  inline T angle() const noexcept {
    return 2 * std::acos(this->w());
  }

  inline Vector3<T> axis() const noexcept {
    const T theta = angle();
    const T sin_ang = std::sin(angle / 2.f);
    if (is_approx(sin_ang, 0.f)) {
      return Vector3<T>(1.0, 0.0 0.0);
    }
    return Vector3<T>(this->x() / sin_ang,  //
                      this->y() / sin_ang,  //
                      this->z() / sin_ang);
  }

  template <typename U>
  inline Quaternion& operator+=(const Quaternion<U>& other) noexcept {
    this->w() += static_cast<T>(other.w());
    this->x() += static_cast<T>(other.x());
    this->y() += static_cast<T>(other.y());
    this->z() += static_cast<T>(other.z());
    return *this;
  }

  template <typename U>
  inline Quaternion& operator-=(const Quaternion<U>& other) noexcept {
    this->w() -= static_cast<T>(other.w());
    this->x() -= static_cast<T>(other.x());
    this->y() -= static_cast<T>(other.y());
    this->z() -= static_cast<T>(other.z());
    return *this;
  }

  template <typename U>
  inline Quaternion& operator*=(U s) noexcept {
    const T tmp_s{static_cast<T>(s)};
    this->w() *= tmp_s;
    this->x() *= tmp_s;
    this->y() *= tmp_s;
    this->z() *= tmp_s;
    return *this;
  }

  template <typename U>
  inline Quaternion& operator*=(const Quaternion<U>& other) noexcept {
    const T w = this->w() * other.w() - this->x() * other.x() -
                this->y() * other.y() - this->z() * other.z();
    const T x = this->w() * other.x() + this->x() * other.w() +
                this->y() * other.z() - this->z() * other.y();
    const T y = this->w() * other.y() - this->x() * other.z() +
                this->y() * other.w() + this->z() * other.x();
    const T z = this->w() * other.z() + this->x() * other.y() -
                this->y() * other.x() + this->z() * other.w();
    this->w() = w;
    this->x() = x;
    this->y() = y;
    this->z() = z;
    return *this;
  }

  inline Quaternion operator*(const Quaternion<U>& other) noexcept {
    Quaternion<T> result(*this);
    result *= other;
    return result;
  }

  inline Vector3<T> operator*(const Vector3<T>& vec) noexcept {
    return matrix() * vec;
  }

  template <typename U>
  inline Quaternion& operator/=(U s) noexcept {
    const T tmp_s{static_cast<T>(s)};
    this->w() /= tmp_s;
    this->x() /= tmp_s;
    this->y() /= tmp_s;
    this->z() /= tmp_s;
    return *this;
  }

  template <typename U>
  inline Quaternion& operator/=(const Quaternion<U>& other) noexcept {
    return this * other.inverse();
  }

  inline T squared_norm() const {
    return this->w() * this->w() +  //
           this->x() * this->x() +  //
           this->y() * this->y() +  //
           this->z() * this->z();
  }

  inline T norm() noexcept {
    return std::sqrt(squared_norm());
  }

  void normalize() noexcept {
    const T n = norm();
    if (is_approx(n, 0.f)) {
      std::wcerr << "WARN: norm is too small: " << n << std::endl;
      return;
    }
    *this /= n;
  }

  Quaternion normalized() const noexcept {
    Quaternion<T> result(*this);
    result.normalize();
    return result;
  }

  void conjugate() noexcept {
    this->x() *= -1;
    this->y() *= -1;
    this->z() *= -1;
  }

  Quaternion conjugated() const noexcept {
    return Quaternion(this->w(), -this->x(), -this->y(), -this->z());
  }

  Quaternion inverse() const noexcept {
    return this->conjugated().normalized();
  }

  bool is_approx(const Quaternion<T>& other) const noexcept {
    return is_approx(this->w(), other.w()) &&  //
           is_approx(this->x(), other.x()) &&  //
           is_approx(this->y(), other.y()) &&  //
           is_approx(this->z(), other.z());
  }

  inline T angular_distance(const Quaternion<T>& other) const noexcept {
    const Quaternion<T> d = (*this) * other.conjugated();
    return 2.0 * std::atan2(d.vector().norm(), std::abs(d.w()));
  }

  Matrix3x3<T> to_matrix() const noexcept {
    Matrix3x3<T> result;
    // const T tx = 2.f * this->x();
    // const T ty = 2.f * this->y();
    // const T tz = 2.f * this->z();
    // const T txx = tx * this->x();
    // const T tyy = ty * this->y();
    // const T tzz = tz * this->z();
    // const T twx = tx * this->w();
    // const T twy = ty * this->w();
    // const T twz = tz * this->w();
    // const T txy = tx * this->y();
    // const T txz = tx * this->z();
    // const T tyz = ty * this->z();
    // result.at(0, 0) = 1.f - tyy - tzz;
    // result.at(0, 1) = txy - twz;
    // result.at(0, 2) = txz + twy;
    // result.at(1, 0) = txy + twz;
    // result.at(1, 1) = 1.f - txx - tzz;
    // result.at(1, 2) = tyz - twx;
    // result.at(2, 0) = txz - twy;
    // result.at(2, 1) = tyz + twx;
    // result.at(2, 2) = 1.f - txx - tyy;
    result.at(0, 0) =
      2.f * (this->w() * this->w() + this->x() * this->x()) - 1.f;
    result.at(0, 1) = 2.f * (this->x() * this->y() - this->w() * this->z());
    result.at(0, 2) = 2.f * (this->x() * this->z() + this->w() * this->y());
    result.at(1, 0) = 2.f * (this->x() * this->y() + this->w() * this->z());
    result.at(1, 1) =
      2.f * (this->w() * this->w() + this->y() * this->y()) - 1.f;
    result.at(1, 2) = 2.f * (this->y() * this->z() - this->w() * this->x());
    result.at(2, 0) = 2.f * (this->x() * this->z() - this->w() * this->y());
    result.at(2, 1) = 2.f * (this->y() * this->z() + this->w() * this->x());
    result.at(2, 2) =
      2.f * (this->w() * this->w() + this->z() * this->z()) - 1.f;
    return result;
  }

  Quaternion<T> log() const noexcept {
    const T n = norm();
    const T result_scale = std::log(n);
    const Vector3<T> result_vec =
      vector().normalized() * std::acos(this->w() / n);
    return Quaternion<T>(result_scale, result_vec.x(), result_vec.y(),
                         result_vec.z());
  }

  Quaternion<T> exp() const noexcept {
    const T gain = std::exp(this->w());
    const Vector3<T> vec = vector();
    const T vn = vec.norm();
    const T result_scale = gain * std::cos(vn);
    if (is_approx(vn, 0)) {
      return Quaternion<T>(result_scale, 0, 0, 0);
    }
    const Vector3<T> result_vec = vec * (gain * std::sin(vn) / vn);
    return Quaternion<T>(result_scale, result_vec);
  }
};

template <typename T>
inline Quaternion<T> operator+(const Quaternion<T>& lhs,
                               const Quaternion<T>& rhs) noexcept {
  Quaternion<T> result(lhs);
  result += rhs;
  return result;
}

template <typename T>
inline Quaternion<T> operator-(const Quaternion<T>& lhs,
                               const Quaternion<T>& rhs) noexcept {
  Quaternion<T> result(lhs);
  result -= rhs;
  return result;
}

template <typename T, typename U>
inline Quaternion<T> operator*(const Quaternion<T>& quat, U s) noexcept {
  Quaternion<T> result(quat);
  result *= s;
  return result;
}

template <typename T>
inline Quaternion<T> operator*(const Quaternion<T>& lhs,
                               const Quaternion<T>& rhs) noexcept {
  Quaternion<T> result(lhs);
  result *= rhs;
  return result;
}

template <typename T, typename U>
inline Quaternion<T> operator/(const Quaternion<T>& quat, U s) noexcept {
  Quaternion<T> result(quat);
  result /= s;
  return result;
}

template <typename T>
inline Quaternion<T> operator/(const Quaternion<T>& lhs,
                               const Quaternion<T>& rhs) noexcept {
  Quaternion<T> result(lhs);
  result /= rhs;
  return result;
}