#pragma once
#include <cstddef>
#include "basic.hpp"
#include "utility.hpp"
#include <cmath>
#include <algorithm>
template <typename T>
struct Vector3 : public VF3<T> {
  Vector3() noexcept : VF3<T>(0.f, 0.f, 0.f) {
  }

  constexpr explicit Vector3(T c) noexcept : VF3<T>(c, c, c) {
  }

  constexpr Vector3(T _x, T _y, T _z) noexcept : VF3<T>(_x, _y, _z) {
  }

  Vector3(const std::array<T, 3>& array) noexcept
    : VF3<T>(array[0], array[1], array[2]) {
  }

  Vector3(const VF3<T>& _v) noexcept {
    this->x = _v.x;
    this->y = _v.y;
    this->z = _v.z;
  }

  Vector3(const Vector3<T>&) = default;
  Vector3(Vector3<T>&&) = default;
  Vector3& operator=(const Vector3<T>&) = default;
  Vector3& operator=(Vector3<T>&&) = default;

  inline static Vector3<T> ones() {
    return Vector3<T>(1.f);
  }

  inline static Vector3<T> zeros() {
    return Vector3<T>(0.f);
  }

  inline static Vector3<T> random(T vmin, T vmax) {
    return Vector3<T>(generate_randoms<T, 3>(vmin, vmax));
  }

  // Comparison operators
  bool operator==(const Vector3<T>& other) const noexcept;
  bool operator!=(const Vector3<T>& other) const noexcept;

  // Assignment operators
  inline Vector3& operator+=(const Vector3<T>& other) noexcept {
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
    return *this;
  }

  inline Vector3& operator-=(const Vector3<T>& other) noexcept {
    this->x -= other.x;
    this->y -= other.y;
    this->z -= other.z;
    return *this;
  }

  inline Vector3& operator*=(const Vector3<T>& other) noexcept {
    this->x *= other.x;
    this->y *= other.y;
    this->z *= other.z;
    return *this;
  }

  inline Vector3& operator*=(T s) noexcept {
    this->x *= s;
    this->y *= s;
    this->z *= s;
    return *this;
  }

  inline Vector3& operator/=(T s) noexcept {
    this->x /= s;
    this->y /= s;
    this->z /= s;
    return *this;
  }

  T sum() const noexcept {
    return this->x + this->y + this->z;
  }

  T dot(const Vector3& other) const noexcept {
    return this->x * other.x +  //
           this->y * other.y +  //
           this->z * other.z;
  }

  Vector3 cross(const Vector3& other) const noexcept {
    const T tmp_x = this->y * other->z - this->z * other->y;
    const T tmp_y = this->z * other->x - this->x * other->z;
    const T tmp_z = this->x * other->y - this->y * other->x;
    return Vector3<T>(tmp_x, tmp_y, tmp_z);
  }

  T norm() noexcept {
    return std::sqrt(dot(*this));
  }

  void normalize() noexcept {
    const T n = norm();
    this->x /= n;
    this->y /= n;
    this->z /= n;
  }

  Vector3 normalized() const noexcept {
    const T n = norm();
    const T tmp_x = this->x / n;
    const T tmp_y = this->y / n;
    const T tmp_z = this->z / n;
    return Vector3<T>(tmp_x, tmp_y, tmp_z);
  }

  void clamp(const Vector3& vmin, const Vector3& vmax) noexcept {
    this->x = std::clamp(vmin->x, vmax->x, this->x);
    this->y = std::clamp(vmin->y, vmax->y, this->y);
    this->z = std::clamp(vmin->z, vmax->z, this->z);
  }

  Vector3 clamped(const Vector3& vmin, const Vector3& vmax) const noexcept {
    const T tmp_x = std::clamp(vmin->x, vmax->x, this->x);
    const T tmp_y = std::clamp(vmin->y, vmax->y, this->y);
    const T tmp_z = std::clamp(vmin->z, vmax->z, this->z);
    return Vector3<T>(tmp_x, tmp_y, tmp_z);
  }

  void clamp(const T& vmin, const T& vmax) noexcept {
    this->x = std::clamp(vmin, vmax, this->x);
    this->y = std::clamp(vmin, vmax, this->y);
    this->z = std::clamp(vmin, vmax, this->z);
  }

  Vector3 clamped(const T& vmin, const T& vmax) const noexcept {
    const T tmp_x = std::clamp(vmin, vmax, this->x);
    const T tmp_y = std::clamp(vmin, vmax, this->y);
    const T tmp_z = std::clamp(vmin, vmax, this->z);
    return Vector3<T>(tmp_x, tmp_y, tmp_z);
  }
};

template <typename T>
inline Vector3<T> operator+(const Vector3<T>& lhs,
                            const Vector3<T>& rhs) noexcept {
  const T tmp_x = lhs.x + rhs.x;
  const T tmp_y = lhs.y + rhs.y;
  const T tmp_z = lhs.z + rhs.z;
  return Vector3<T>(tmp_x, tmp_y, tmp_z);
}

template <typename T>
inline Vector3<T> operator-(const Vector3<T>& lhs,
                            const Vector3<T>& rhs) noexcept {
  const T tmp_x = lhs.x - rhs.x;
  const T tmp_y = lhs.y - rhs.y;
  const T tmp_z = lhs.z - rhs.z;
  return Vector3(tmp_x, tmp_y, tmp_z);
}

template <typename T>
inline Vector3<T> operator*(const Vector3<T>& lhs,
                            const Vector3<T>& rhs) noexcept {
  const T tmp_x = lhs.x * rhs.x;
  const T tmp_y = lhs.y * rhs.y;
  const T tmp_z = lhs.z * rhs.z;
  return Vector3(tmp_x, tmp_y, tmp_z);
}

template <typename T>
inline Vector3<T> operator*(const Vector3<T>& vec, T s) noexcept {
  return Vector3(vec.x * s, vec.y * s, vec.z * s);
}

template <typename T>
inline Vector3<T> operator/(const Vector3<T>& vec, T s) noexcept {
  return Vector3(vec.x / s, vec.y / s, vec.z / s);
}