// Copyright (C) 2024 Bara Emran - All Rights Reserved

#pragma once
#include <cstddef>
#include "basic.hpp"
#include "utility.hpp"
#include <ostream>
#include <array>
#include <sstream>
#include <iostream>

template <typename T>
struct Matrix2x2 : public BasicMatrix2x2<T> {
  Matrix2x2() noexcept : BasicMatrix2x2<T>() {
  }

  explicit Matrix2x2(T c) noexcept : BasicMatrix2x2<T>(c) {
  }

  Matrix2x2(T m00, T m01, T m10, T m11) noexcept
    : BasicMatrix2x2<T>(m00, m01, m10, m11) {
  }

  Matrix2x2(const std::array<T, 4>& array) noexcept
    : BasicMatrix2x2<T>(array[0], array[1], array[2], array[3]) {
  }

  Matrix2x2(const BasicMatrix2x2<T>& bm) noexcept : BasicMatrix2x2<T>(bm) {
  }

  // Matrix2x2(const Matrix2x2<T>&) = default;
  // Matrix2x2(Matrix2x2<T>&&) = default;
  // Matrix2x2& operator=(const Matrix2x2<T>&) = default;
  // Matrix2x2& operator=(Matrix2x2<T>&&) = default;

  inline static Matrix2x2<T> eye() {
    return Matrix2x2<T>(1.f, 0.f, 0.f, 1.f);
  }

  inline static Matrix2x2<T> ones() {
    return Matrix2x2<T>(1.f);
  }

  inline static Matrix2x2<T> zeros() {
    return Matrix2x2<T>(0.f);
  }

  inline static Matrix2x2<T> random(T vmin, T vmax) {
    return Matrix2x2<T>(generate_randoms<T, 4>(vmin, vmax));
  }

  // // Comparison operators
  // bool operator==(const Matrix2x2<T>& other) const noexcept {
  //   return                       //
  //     this->mat[0][0] == other(0, 0) &&  //
  //     this->mat[0][1] == other(0, 1) &&  //
  //     this->mat[1][0] == other(1, 0) &&  //
  //     this->mat[1][1] == other(1, 1);
  // }

  // bool operator!=(const Matrix2x2<T>& other) const noexcept {
  //   return not this->operator==(other);
  // }

  // // Assignment operators
  // Matrix2x2& operator=(const BasicMatrix2x2<T>& other) noexcept {
  //   this->mat[0][0] = other(0, 0);
  //   this->mat[0][1] = other(0, 1);
  //   this->mat[1][0] = other(1, 0);
  //   this->mat[1][1] = other(1, 1);
  //   return *this;
  // }

  Matrix2x2& operator+=(const Matrix2x2<T>& other) noexcept {
    this->mat[0][0] += other(0, 0);
    this->mat[0][1] += other(0, 1);
    this->mat[1][0] += other(1, 0);
    this->mat[1][1] += other(1, 1);
    return *this;
  }

  Matrix2x2<T>& operator-=(const Matrix2x2<T>& other) noexcept {
    this->mat[0][0] -= other(0, 0);
    this->mat[0][1] -= other(0, 1);
    this->mat[1][0] -= other(1, 0);
    this->mat[1][1] -= other(1, 1);
    return *this;
  }

  Matrix2x2<T>& operator*=(T s) noexcept {
    this->mat[0][0] *= s;
    this->mat[0][1] *= s;
    this->mat[1][0] *= s;
    this->mat[1][1] *= s;
    return *this;
  }

  Matrix2x2<T>& operator*=(const Matrix2x2<T>& other) noexcept {
    // this is done in this way because of one used for the same matrix (mat *=
    // mat) it will update the matrix while in the middle of multiplication
    const T m00 = this->mat[0][0] * other(0, 0) + this->mat[0][1] * other(1, 0);
    const T m01 = this->mat[0][0] * other(0, 1) + this->mat[0][1] * other(1, 1);
    const T m10 = this->mat[1][0] * other(0, 0) + this->mat[1][1] * other(1, 0);
    const T m11 = this->mat[1][0] * other(0, 1) + this->mat[1][1] * other(1, 1);
    this->mat[0][0] = m00;
    this->mat[0][1] = m01;
    this->mat[1][0] = m10;
    this->mat[1][1] = m11;
    return *this;
  }

  Matrix2x2<T>& operator/=(T s) noexcept {
    this->mat[0][0] /= s;
    this->mat[0][1] /= s;
    this->mat[1][0] /= s;
    this->mat[1][1] /= s;
    return *this;
  }

  Matrix2x2<T>& operator/=(const Matrix2x2<T>& other) noexcept {
    this->operator*=(other.inversed());
    return *this;
  }

  void transpose() noexcept {
    std::swap(this->mat[0][1], this->mat[1][0]);
  }

  Matrix2x2 transposed() const noexcept {
    Matrix2x2 result(*this);
    result.transpose();
    return result;
  }

  T det() const noexcept {
    return this->mat[0][0] * this->mat[1][1] -
           this->mat[0][1] * this->mat[1][0];
  }

  void inverse() noexcept {
    const T d = det();
    const float tmp = this->mat[0][0];
    this->mat[0][0] = this->mat[1][1] / d;
    this->mat[0][1] /= -d;
    this->mat[1][0] /= -d;
    this->mat[1][1] = tmp / d;
  }

  Matrix2x2 adjoint() const noexcept {
    // Calculate the cofactor matrix and transpose matrix it at the same time
    return Matrix2x2(this->mat[1][1], this->mat[0][1], this->mat[1][0], this->mat[0][0]);
  }

  void clamp(T vmin, T vmax) noexcept {
    this->mat[0][0] = std::clamp(this->mat[0][0], vmin, vmax);
    this->mat[0][1] = std::clamp(this->mat[0][1], vmin, vmax);
    this->mat[1][0] = std::clamp(this->mat[1][0], vmin, vmax);
    this->mat[1][1] = std::clamp(this->mat[1][1], vmin, vmax);
  }

  Matrix2x2 clamped(T vmin, T vmax) const noexcept {
    Matrix2x2<T> result(*this);
    result.clamp(vmin, vmax);
    return std::move(result);
  }
};

template <typename T>
Matrix2x2<T> operator+(const Matrix2x2<T>& lhs,
                       const Matrix2x2<T>& rhs) noexcept {
  Matrix2x2<T> result(lhs);
  result += rhs;
  return result;
}

template <typename T>
Matrix2x2<T> operator-(const Matrix2x2<T>& lhs,
                       const Matrix2x2<T>& rhs) noexcept {
  Matrix2x2<T> result(lhs);
  result -= rhs;
  return result;
}

template <typename T>
Matrix2x2<T> operator*(const Matrix2x2<T>& lhs,
                       const Matrix2x2<T>& rhs) noexcept {
  Matrix2x2<T> result(lhs);
  result *= rhs;
  return result;
}

template <typename T>
Matrix2x2<T> operator*(const Matrix2x2<T>& vec, T s) noexcept {
  Matrix2x2<T> result(vec);
  result *= s;
  return result;
}

template <typename T>
Matrix2x2<T> operator/(const Matrix2x2<T>& vec, T s) noexcept {
  Matrix2x2<T> result(vec);
  result /= s;
  return result;
}

template <typename T>
Matrix2x2<T> operator/(const Matrix2x2<T>& lhs,
                       const Matrix2x2<T>& rhs) noexcept {
  Matrix2x2<T> result(lhs);
  result /= rhs;
  return result;
}