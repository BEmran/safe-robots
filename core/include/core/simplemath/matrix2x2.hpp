// Copyright (C) 2024 Bara Emran - All Rights Reserved

#pragma once
#include "core/simplemath/basic.hpp"
#include "core/simplemath/vector2.hpp"
#include "core/simplemath/utility.hpp"

#include <array>
#include <cstddef>
#include <iostream>

template <typename T>
struct Matrix2x2 : public BasicMatrix2x2<T> {
  using BasicMatrix2x2<T>::data;
  using BasicMatrix2x2<T>::mat;

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

  bool is_approx(const Matrix2x2<T>& other) const noexcept {
    return is_approx(data[0], other.at(0)) &&  //
           is_approx(data[1], other.at(1)) &&  //
           is_approx(data[2], other.at(2)) &&  //
           is_approx(data[3], other.at(3));
  }

  Matrix2x2& operator+=(const Matrix2x2<T>& other) noexcept {
    mat[0][0] += other.at(0, 0);
    mat[0][1] += other.at(0, 1);
    mat[1][0] += other.at(1, 0);
    mat[1][1] += other.at(1, 1);
    return *this;
  }

  Matrix2x2<T>& operator-=(const Matrix2x2<T>& other) noexcept {
    mat[0][0] -= other.at(0, 0);
    mat[0][1] -= other.at(0, 1);
    mat[1][0] -= other.at(1, 0);
    mat[1][1] -= other.at(1, 1);
    return *this;
  }

  Matrix2x2<T>& operator*=(T s) noexcept {
    mat[0][0] *= s;
    mat[0][1] *= s;
    mat[1][0] *= s;
    mat[1][1] *= s;
    return *this;
  }

  Matrix2x2<T>& operator*=(const Matrix2x2<T>& other) noexcept {
    // this is done in this way because of one used for the same matrix (mat *=
    // mat) it will update the matrix while in the middle of multiplication

    auto row_by_col = [&](size_t row, size_t col) -> T {
      return mat[row][0] * other.at(0, col) + mat[row][1] * other.at(1, col);
    };
    T result[2][2];
    result[0][0] = row_by_col(0, 0);
    result[0][1] = row_by_col(0, 1);
    result[1][0] = row_by_col(1, 0);
    result[1][1] = row_by_col(1, 1);
    mat[0][0] = result[0][0];
    mat[0][1] = result[0][1];
    mat[1][0] = result[1][0];
    mat[1][1] = result[1][1];
    return *this;
  }

  Matrix2x2<T>& operator/=(T s) noexcept {
    mat[0][0] /= s;
    mat[0][1] /= s;
    mat[1][0] /= s;
    mat[1][1] /= s;
    return *this;
  }

  Matrix2x2<T>& operator/=(const Matrix2x2<T>& other) noexcept {
    this->operator*=(other.inverse());
    return *this;
  }

  void transpose() noexcept {
    std::swap(mat[0][1], mat[1][0]);
  }

  Matrix2x2 transposed() const noexcept {
    Matrix2x2 result(*this);
    result.transpose();
    return result;
  }

  T det() const noexcept {
    return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
  }

  Matrix2x2 inverse() noexcept {
    static constexpr T EPSILON{static_cast<T>(0.00001)};
    const T d = det();
    if (d < EPSILON && d > -EPSILON) {
      std::wcerr << "WARN: determinant is too small: " << d << std::endl;
      return *this;  // TODO: print warning when result is false
    }
    return adjoint() / d;
  }

  Matrix2x2 adjoint() const noexcept {
    // Calculate the cofactor matrix and transpose matrix it at the same time
    return Matrix2x2(mat[1][1], -mat[0][1], -mat[1][0], mat[0][0]);
  }

  void clamp(T vmin, T vmax) noexcept {
    mat[0][0] = std::clamp(mat[0][0], vmin, vmax);
    mat[0][1] = std::clamp(mat[0][1], vmin, vmax);
    mat[1][0] = std::clamp(mat[1][0], vmin, vmax);
    mat[1][1] = std::clamp(mat[1][1], vmin, vmax);
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