#pragma once
#include "core/simplemath/basic.hpp"
#include "core/simplemath/utility.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>

template <typename T>
struct Vector3 : public BasicVector3<T> {
  Vector3() noexcept : BasicVector3<T>() {
  }

  explicit Vector3(T c) noexcept : BasicVector3<T>(c) {
  }

  Vector3(T x, T y, T z) noexcept : BasicVector3<T>(x, y, z) {
  }

  Vector3(const std::array<T, 3>& array) noexcept
    : BasicVector3<T>(array[0], array[1], array[2]) {
  }

  Vector3(const BasicVector3<T>& bv) noexcept : BasicVector3<T>(bv) {
  }

  inline static Vector3<T> ones() {
    return Vector3<T>(1.f);
  }

  inline static Vector3<T> zeros() {
    return Vector3<T>(0.f);
  }

  inline static Vector3<T> random(T vmin, T vmax) {
    return Vector3<T>(generate_randoms<T, 3>(vmin, vmax));
  }

  bool is_approx(const Vector3<T>& other) const noexcept {
    return is_approx(this->x(), other.x()) &&  //
           is_approx(this->y(), other.y()) &&  //
           is_approx(this->z(), other.z());
  }

  inline Vector3& operator+=(const Vector3<T>& other) noexcept {
    this->x() += other.x();
    this->y() += other.y();
    this->z() += other.z();
    return *this;
  }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wself-assign-overloaded"
  inline Vector3& operator-=(const Vector3<T>& other) noexcept {
    this->x() -= other.x();
    this->y() -= other.y();
    this->z() -= other.z();
    return *this;
  }
#pragma GCC diagnostic pop

  inline Vector3& operator*=(const Vector3<T>& other) noexcept {
    this->x() *= other.x();
    this->y() *= other.y();
    this->z() *= other.z();
    return *this;
  }

  inline Vector3& operator*=(T s) noexcept {
    this->x() *= s;
    this->y() *= s;
    this->z() *= s;
    return *this;
  }

  inline Vector3& operator/=(T s) noexcept {
    this->x() /= s;
    this->y() /= s;
    this->z() /= s;
    return *this;
  }

  T sum() const noexcept {
    return this->x() + this->y() + this->z();
  }

  T dot(const Vector3& other) const noexcept {
    return this->x() * other.x() +  //
           this->y() * other.y() +  //
           this->z() * other.z();
  }

  Vector3 cross(const Vector3& other) const noexcept {
    const T tmp_x = this->y() * other.z() - this->z() * other.y();
    const T tmp_y = this->z() * other.x() - this->x() * other.z();
    const T tmp_z = this->x() * other.y() - this->y() * other.x();
    return Vector3<T>(tmp_x, tmp_y, tmp_z);
  }

  T norm() noexcept {
    return std::sqrt(dot(*this));
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

  Vector3 normalized() const noexcept {
    Vector3<T> result(*this);
    result.normalize();
    return result;
  }

  void clamp(const T& vmin, const T& vmax) noexcept {
    this->x() = std::clamp(this->x(), vmin, vmax);
    this->y() = std::clamp(this->y(), vmin, vmax);
    this->z() = std::clamp(this->z(), vmin, vmax);
  }

  void clamp(const Vector3& vmin, const Vector3& vmax) noexcept {
    this->x() = std::clamp(this->x(), vmin.x(), vmax.x());
    this->y() = std::clamp(this->y(), vmin.y(), vmax.y());
    this->z() = std::clamp(this->z(), vmin.z(), vmax.z());
  }

  Vector3 clamped(const T& vmin, const T& vmax) const noexcept {
    Vector3<T> result(*this);
    result.clamp(vmin, vmax);
    return result;
  }

  Vector3 clamped(const Vector3& vmin, const Vector3& vmax) const noexcept {
    Vector3<T> result(*this);
    result.clamp(vmin, vmax);
    return result;
  }
};

template <typename T>
inline Vector3<T> operator+(const Vector3<T>& lhs,
                            const Vector3<T>& rhs) noexcept {
  Vector3<T> result(lhs);
  result += rhs;
  return result;
}

template <typename T>
inline Vector3<T> operator-(const Vector3<T>& lhs,
                            const Vector3<T>& rhs) noexcept {
  Vector3<T> result(lhs);
  result -= rhs;
  return result;
}

template <typename T>
inline Vector3<T> operator*(const Vector3<T>& lhs,
                            const Vector3<T>& rhs) noexcept {
  Vector3<T> result(lhs);
  result *= rhs;
  return result;
}

template <typename T>
inline Vector3<T> operator*(const Vector3<T>& vec, T s) noexcept {
  Vector3<T> result(vec);
  result *= s;
  return result;
}

template <typename T>
inline Vector3<T> operator/(const Vector3<T>& vec, T s) noexcept {
  Vector3<T> result(vec);
  result /= s;
  return result;
}