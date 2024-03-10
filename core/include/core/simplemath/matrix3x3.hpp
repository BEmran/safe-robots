// Copyright (C) 2024 Bara Emran - All Rights Reserved

#pragma once
#include "core/simplemath/basic.hpp"
#include "core/simplemath/vector3.hpp"
#include "core/simplemath/utility.hpp"

#include <array>
#include <cstddef>
#include <iostream>

template <typename T>
struct Matrix3x3 : public BasicMatrix3x3<T> {
  using BasicMatrix3x3<T>::data;
  using BasicMatrix3x3<T>::mat;
  Matrix3x3() noexcept : BasicMatrix3x3<T>() {
  }

  explicit Matrix3x3(T c) noexcept : BasicMatrix3x3<T>(c) {
  }

  Matrix3x3(T m00, T m01, T m02, T m10, T m11, T m12, T m20, T m21,
            T m22) noexcept
    : BasicMatrix3x3<T>(m00, m01, m02, m10, m11, m12, m20, m21, m22) {
  }

  Matrix3x3(const std::array<T, 9>& array) noexcept
    : BasicMatrix3x3<T>(array[0], array[1], array[2], array[3], array[4],
                        array[5], array[6], array[7], array[8]) {
  }

  Matrix3x3(const BasicMatrix3x3<T>& bm) noexcept : BasicMatrix3x3<T>(bm) {
  }

  inline static Matrix3x3<T> eye() {
    return Matrix3x3<T>(1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f);
  }

  inline static Matrix3x3<T> ones() {
    return Matrix3x3<T>(1.f);
  }

  inline static Matrix3x3<T> zeros() {
    return Matrix3x3<T>(0.f);
  }

  inline static Matrix3x3<T> random(T vmin, T vmax) {
    return Matrix3x3<T>(generate_randoms<T, 9>(vmin, vmax));
  }

  Matrix3x3& operator+=(const Matrix3x3<T>& other) noexcept {
    data[0] += other.at(0);
    data[1] += other.at(1);
    data[2] += other.at(2);
    data[3] += other.at(3);
    data[4] += other.at(4);
    data[5] += other.at(5);
    data[6] += other.at(6);
    data[7] += other.at(7);
    data[8] += other.at(8);
    return *this;
  }

  Matrix3x3<T>& operator-=(const Matrix3x3<T>& other) noexcept {
    data[0] -= other.at(0);
    data[1] -= other.at(1);
    data[2] -= other.at(2);
    data[3] -= other.at(3);
    data[4] -= other.at(4);
    data[5] -= other.at(5);
    data[6] -= other.at(6);
    data[7] -= other.at(7);
    data[8] -= other.at(8);
    return *this;
  }

  Matrix3x3<T>& operator*=(const Matrix3x3<T>& other) noexcept {
    auto row_by_col = [&](size_t row, size_t col) -> T {
      return mat[row][0] * other.at(0, col) +  //
             mat[row][1] * other.at(1, col) +  //
             mat[row][2] * other.at(2, col);
    };

    T result[3][3];
    result[0][0] = row_by_col(0, 0);
    result[1][0] = row_by_col(1, 0);
    result[2][0] = row_by_col(2, 0);
    result[0][1] = row_by_col(0, 1);
    result[1][1] = row_by_col(1, 1);
    result[2][1] = row_by_col(2, 1);
    result[0][2] = row_by_col(0, 2);
    result[1][2] = row_by_col(1, 2);
    result[2][2] = row_by_col(2, 2);

    mat[0][0] = result[0][0];
    mat[0][1] = result[0][1];
    mat[0][2] = result[0][2];
    mat[1][0] = result[1][0];
    mat[1][1] = result[1][1];
    mat[1][2] = result[1][2];
    mat[2][0] = result[2][0];
    mat[2][1] = result[2][1];
    mat[2][2] = result[2][2];

    return *this;
  }

  Matrix3x3<T>& operator*=(T s) noexcept {
    data[0] *= s;
    data[1] *= s;
    data[2] *= s;
    data[3] *= s;
    data[4] *= s;
    data[5] *= s;
    data[6] *= s;
    data[7] *= s;
    data[8] *= s;
    return *this;
  }

  Matrix3x3<T>& operator/=(T s) noexcept {
    data[0] /= s;
    data[1] /= s;
    data[2] /= s;
    data[3] /= s;
    data[4] /= s;
    data[5] /= s;
    data[6] /= s;
    data[7] /= s;
    data[8] /= s;
    return *this;
  }

  Matrix3x3<T>& operator/=(const Matrix3x3<T>& other) noexcept {
    this->operator*=(other.inverse());
    return *this;
  }

  T det() const noexcept {
    return mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) +
           mat[0][1] * (mat[2][0] * mat[1][2] - mat[1][0] * mat[2][2]) +
           mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
  }

  void transpose() noexcept {
    std::swap(mat[0][1], mat[1][0]);
    std::swap(mat[0][2], mat[2][0]);
    std::swap(mat[1][2], mat[2][1]);
  }

  Matrix3x3 transposed() const noexcept {
    Matrix3x3 result(*this);
    result.transpose();
    return result;
  }

  Matrix3x3 inverse() noexcept {
    static constexpr T EPSILON{static_cast<T>(0.00001)};
    const T d = det();
    if (d < EPSILON && d > -EPSILON) {
      std::wcerr << "WARN: det is too small: " << d << std::endl;
      return *this;  // TODO: print warning when result is false
    }
    return adjoint() / d;
  }

  Matrix3x3 adjoint() const noexcept {
    // Calculate the cofactor matrix and transpose matrix it at the same time.
    const T m00 = mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1];
    const T m01 = mat[2][0] * mat[1][2] - mat[1][0] * mat[2][2];
    const T m02 = mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0];
    const T m10 = mat[2][1] * mat[0][2] - mat[0][1] * mat[2][2];
    const T m11 = mat[0][0] * mat[2][2] - mat[2][0] * mat[0][2];
    const T m12 = mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1];
    const T m20 = mat[0][1] * mat[1][2] - mat[1][1] * mat[0][2];
    const T m21 = mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2];
    const T m22 = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
    return Matrix3x3(m00, m10, m20, m01, m11, m21, m02, m12,
                     m22);  // +transpose
  }

  void clamp(T vmin, T vmax) noexcept {
    data[0] = std::clamp(data[0], vmin, vmax);
    data[1] = std::clamp(data[1], vmin, vmax);
    data[2] = std::clamp(data[2], vmin, vmax);
    data[3] = std::clamp(data[3], vmin, vmax);
    data[4] = std::clamp(data[4], vmin, vmax);
    data[5] = std::clamp(data[5], vmin, vmax);
    data[6] = std::clamp(data[6], vmin, vmax);
    data[7] = std::clamp(data[7], vmin, vmax);
    data[8] = std::clamp(data[8], vmin, vmax);
  }

  Matrix3x3 clamped(T vmin, T vmax) const noexcept {
    Matrix3x3<T> result(*this);
    result.clamp(vmin, vmax);
    return result;
  }

  inline Vector3<T> row(size_t idx) const noexcept {
    return Vector3<T>(mat[idx][0], mat[idx][1], mat[idx][2]);
  }

  inline Vector3<T> col(size_t idx) const noexcept {
    return Vector3<T>(mat[0][idx], mat[1][idx], mat[2][idx]);
  }
};

template <typename T>
Matrix3x3<T> operator+(const Matrix3x3<T>& lhs,
                       const Matrix3x3<T>& rhs) noexcept {
  Matrix3x3<T> result(lhs);
  result += rhs;
  return result;
}

template <typename T>
Matrix3x3<T> operator-(const Matrix3x3<T>& lhs,
                       const Matrix3x3<T>& rhs) noexcept {
  Matrix3x3<T> result(lhs);
  result -= rhs;
  return result;
}

template <typename T>
Matrix3x3<T> operator*(const Matrix3x3<T>& lhs,
                       const Matrix3x3<T>& rhs) noexcept {
  Matrix3x3<T> result(lhs);
  result *= rhs;
  return result;
}

template <typename T>
Matrix3x3<T> operator*(const Matrix3x3<T>& vec, T s) noexcept {
  Matrix3x3<T> result(vec);
  result *= s;
  return result;
}

template <typename T>
inline Matrix3x3<T> operator/(const Matrix3x3<T>& vec, T s) noexcept {
  Matrix3x3<T> result(vec);
  result /= s;
  return result;
}

template <typename T>
inline Matrix3x3<T> operator/(const Matrix3x3<T>& lhs,
                              const Matrix3x3<T>& rhs) noexcept {
  Matrix3x3<T> result(lhs);
  result /= rhs;
  return result;
}
