#pragma once
#include "basic.hpp"

#include <cstddef>
#include "utility.hpp"
#include <cmath>
#include <algorithm>

template <typename T>
struct Vector2 : public BasicVector2<T>{
  Vector2() noexcept : BasicVector2<T>() {
  }

  explicit Vector2(T c) noexcept : BasicVector2<T>(c) {
  }

  Vector2(T x, T y) noexcept : BasicVector2<T>(x, y) {
  }

  Vector2(const std::array<T, 2>& array) noexcept
    : BasicVector2<T>(array[0], array[1]) {
  }

  Vector2(const BasicVector2<T>& bv) noexcept : BasicVector2<T>(bv) {
  }

  // Vector2(const Vector2<T>&) = default;
  // Vector2(Vector2<T>&&) = default;
  // Vector2& operator=(const Vector2<T>&) = default;
  // Vector2& operator=(Vector2<T>&&) = default;

  inline static Vector2<T> ones() {
    return Vector2<T>(1.f);
  }

  inline static Vector2<T> zeros() {
    return Vector2<T>(0.f);
  }

  inline static Vector2<T> random(T vmin, T vmax) {
    return Vector2<T>(generate_randoms<T, 2>(vmin, vmax));
  }

  // // Comparison operators
  // bool operator==(const Vector2<T>& other) const noexcept;
  // bool operator!=(const Vector2<T>& other) const noexcept;

  // Assignment operators
  inline Vector2& operator+=(const Vector2<T>& other) noexcept {
    this->x() += other.x();
    this->y() += other.y();
    return *this;
  }

  inline Vector2& operator-=(const Vector2<T>& other) noexcept {
    this->x() -= other.x();
    this->y() -= other.y();
    return *this;
  }

  inline Vector2& operator*=(const Vector2<T>& other) noexcept {
    this->x() *= other.x();
    this->y() *= other.y();
    return *this;
  }

  inline Vector2& operator*=(T s) noexcept {
    this->x() *= s;
    this->y() *= s;
    return *this;
  }

  inline Vector2& operator/=(T s) noexcept {
    this->x() /= s;
    this->y() /= s;
    return *this;
  }

  T sum() const noexcept {return this->x() + this->y();}

  T dot(const Vector2& other) const noexcept {
    return this->x() * other.x() +  //
           this->y() * other.y();
  }

  Vector2 cross(const Vector2& other) const noexcept {
    // const T tmp_x =
    //   this->y() * other->z() - this->z() * other->y();
    // const T tmp_y =
    //   this->z() * other->x() - this->x() * other->z();
    // const T tmp_z =
    //   this->x() * other->y() - this->y() * other->x();
    return other;
  }

  T norm() const noexcept {
    return static_cast<T>(std::sqrt(dot(*this)));
  }

  void normalize() noexcept {
    static constexpr double EPSILON{0.00001};
    const T n = norm();
    if (static_cast<double>(n) < EPSILON) {
      // TODO: print warning when result is false
      return;
    }
    *this /= n;
  }

  Vector2 normalized() const noexcept {
    Vector2<T> result(*this);
    result.normalize();
    return result;
  }

  void clamp(const T& vmin, const T& vmax) noexcept {
    this->x() = std::clamp(this->x(), vmin, vmax);
    this->y() = std::clamp(this->y(), vmin, vmax);
  }

  void clamp(const Vector2& vmin, const Vector2& vmax) noexcept {
    this->x() = std::clamp(this->x(), vmin.x(), vmax.x());
    this->y() = std::clamp(this->y(), vmin.y(), vmax.y());
  }

  Vector2 clamped(const T& vmin, const T& vmax) const noexcept {
    Vector2<T> result(*this);
    result.clamp(vmin, vmax);
    return result;
  }

  Vector2 clamped(const Vector2& vmin, const Vector2& vmax) const noexcept {
    Vector2<T> result(*this);
    result.clamp(vmin, vmax);
    return result;
  }
};

template <typename T>
inline Vector2<T> operator+(const Vector2<T>& lhs,
                            const Vector2<T>& rhs) noexcept {
  Vector2<T> result(lhs);
  result += rhs;
  return result;
}

template <typename T>
inline Vector2<T> operator-(const Vector2<T>& lhs,
                            const Vector2<T>& rhs) noexcept {
  Vector2<T> result(lhs);
  result -= rhs;
  return result;
}

template <typename T>
inline Vector2<T> operator*(const Vector2<T>& lhs,
                            const Vector2<T>& rhs) noexcept {
  Vector2<T> result(lhs);
  result *= rhs;
  return result;
}

template <typename T>
inline Vector2<T> operator*(const Vector2<T>& vec, T s) noexcept {
  Vector2<T> result(vec);
  result *= s;
  return result;
}

template <typename T>
inline Vector2<T> operator/(const Vector2<T>& vec, T s) noexcept {
  Vector2<T> result(vec);
  result /= s;
  return result;
}