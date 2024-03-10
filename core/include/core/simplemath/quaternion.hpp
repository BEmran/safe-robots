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

  Quaternion(T w, const Vector3<T>& vec) noexcept
    : BasicVector4<T>(w, vec.x(), vec.y(), vec.z()) {
  }

  inline static Quaternion random(T vmin, T vmax) {
    return Quaternion<T>(generate_randoms<T, 4>(vmin, vmax));
  }

  template <typename U>
  inline Quaternion& operator+=(const Quaternion<U>& other) noexcept {
    this->w() += other->w();
    this->x() += other->x();
    this->y() += other->y();
    this->z() += other->z();
    return *this;
  }

  template <typename U>
  inline Quaternion& operator-=(const Quaternion<U>& other) noexcept {
    this->w() -= other->w();
    this->x() -= other->x();
    this->y() -= other->y();
    this->z() -= other->z();
    return *this;
  }

  template <typename U>
  inline Quaternion& operator*=(U s) noexcept {
    this->w() *= s;
    this->x() *= s;
    this->y() *= s;
    this->z() *= s;
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

  template <typename U>
  inline Quaternion& operator/=(U s) noexcept {
    this->w() /= s;
    this->x() /= s;
    this->y() /= s;
    this->z() /= s;
    return *this;
  }

  template <typename U>
  inline Quaternion& operator/=(const Quaternion<U>& other) noexcept {
    return this->operator*=(other->inverse());
  }

  T norm() noexcept {
    return std::sqrt(this->w() * this->w() + this->x() * this->x() +
                     this->y() * this->y() + this->z() * this->z());
  }

  void normalize() noexcept {
    static constexpr T EPSILON{static_cast<T>(0.00001)};
    const T n = norm();
    if (n < EPSILON && n > -EPSILON) {
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
    return Quaternion(-this->x(), -this->y(), -this->z(), this->w());
  }

  Quaternion inverse() const noexcept {
    return this->conjugated().normalized();
  }

  static Quaternion from_tb(const Vector3<T>& tb) {
    const T tb_2[3];
    tb_2[0] = tb.x() / 2.0;
    tb_2[1] = tb.y() / 2.0;
    tb_2[2] = tb.z() / 2.0;
    const T cos_x2 = std::cos(tb_2[0]);
    const T sin_x2 = std::sin(tb_2[0]);
    const T cos_y2 = std::cos(tb_2[1]);
    const T sin_y2 = std::sin(tb_2[1]);
    const T cos_z2 = std::cos(tb_2[2]);
    const T sin_z2 = std::sin(tb_2[2]);
    Quaternion result;
    result.w() = cos_x2 * cos_y2 * cos_z2 + sin_x2 * sin_y2 * sin_z2;
    result.x() = sin_x2 * cos_y2 * cos_z2 - cos_x2 * sin_y2 * sin_z2;
    result.y() = cos_x2 * sin_y2 * cos_z2 + sin_x2 * cos_y2 * sin_z2;
    result.z() = cos_x2 * cos_y2 * sin_z2 - sin_x2 * sin_y2 * cos_z2;
    return result.normalized();
  }

  static Quaternion from_angle_and_axis(T rad, const Vector3<T>& axis) {
    const T c = std::cos(rad / 2);
    const T s = std::sin(rad / 2);
    return Quaternion(c, axis * s);
  }

  static Quaternion from_matrix(const Matrix3x3<T>& mat) {
    const T m00 = mat.at(0, 0);
    const T m11 = mat.at(1, 1);
    const T m22 = mat.at(2, 2);
    const T tr = m00 + m11 + m22;
    Quaternion result;

    if (tr > 0) {
      const T S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
      result.w() = 0.25 * S;
      result.x() = (mat.at(2, 1) - mat.at(1, 2)) / S;
      result.y() = (mat.at(0, 2) - mat.at(2, 0)) / S;
      result.z() = (mat.at(1, 0) - mat.at(0, 1)) / S;
    } else if ((m00 > m11) & (m00 > m22)) {
      const T S = sqrt(1.0 + m00 - m11 - m22) * 2.0;  // S=4*qx
      result.w() = (mat.at(2, 1) - mat.at(12)) / S;
      result.x() = 0.25 * S;
      result.y() = (mat.at(0, 1) + mat.at(10)) / S;
      result.z() = (mat.at(0, 2) + mat.at(20)) / S;
    } else if (m11 > m22) {
      const T S = sqrt(1.0 + m11 - m00 - m22) * 2.0;  // S=4*qy
      result.w() = (mat.at(0, 2) - mat.at(2, 0)) / S;
      result.x() = (mat.at(0, 1) + mat.at(1, 0)) / S;
      result.y() = 0.25 * S;
      result.z() = (mat.at(1, 2) + mat.at(2, 1)) / S;
    } else {
      const T S = sqrt(1.0 + m22 - m00 - m11) * 2.0;  // S=4*qz
      result.w() = (mat.at(1, 0) - mat.at(0, 1)) / S;
      result.x() = (mat.at(0, 2) + mat.at(2, 0)) / S;
      result.y() = (mat.at(1, 2) + mat.at(2, 1)) / S;
      result.z() = 0.25 * S;
    }
    return result;
  }

  inline T scalar() const noexcept {
    return this->w();
  }

  inline Vector3<T> vec() const noexcept {
    return Vector3<T>(this->x(), this->y(), this->z());
  }

  Matrix3x3<T> matrix() const noexcept {
    Matrix3x3<T> result;
    // First row of the rotation matrix
    result.at(0, 0) = 2.f * (this->w() * this->w() + this->x() * this->x()) - 1.f;
    result.at(0, 1) = 2.f * (this->x() * this->y() - this->w() * this->z());
    result.at(0, 2) = 2.f * (this->x() * this->z() + this->w() * this->y());
    result.at(1, 0) = 2.f * (this->x() * this->y() + this->w() * this->z());
    result.at(1, 1) = 2.f * (this->w() * this->w() + this->y() * this->y()) - 1.f;
    result.at(1, 2) = 2.f * (this->y() * this->z() - this->w() * this->x());
    result.at(2, 0) = 2.f * (this->x() * this->z() - this->w() * this->y());
    result.at(2, 1) = 2.f * (this->y() * this->z() + this->w() * this->x());
    result.at(2, 2) = 2.f * (this->w() * this->w() + this->z() * this->z()) - 1.f;
    return result;
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

template <typename T>
inline Quaternion<T> operator*(const Quaternion<T>& lhs,
                               const Quaternion<T>& rhs) noexcept {
  Quaternion<T> result(lhs);
  result *= rhs;
  return result;
}

template <typename T, typename U>
inline Quaternion<T> operator*(const Quaternion<T>& quat, U s) noexcept {
  Quaternion<T> result(quat);
  result *= s;
  return result;
}

template <typename T>
inline Quaternion<T> operator/(const Quaternion<T>& quat, T s) noexcept {
  Quaternion<T> result(quat);
  result /= s;
  return result;
}

template <typename T, typename U>
inline Quaternion<T> operator/(const Quaternion<T>& lhs, U s) noexcept {
  Quaternion<T> result(lhs);
  result /= s;
  return result;
}

template <typename T>
Quaternion<T> rotate(const Quaternion<T>& lhs, const Quaternion<T>& rhs) {
  return rhs * lhs * rhs.conjugated();
}

template <typename T>
Vector3<T> rotate(const Quaternion<T>& quat, const Vector3<T>& vec) {
  return quat.matrix() * vec;
}