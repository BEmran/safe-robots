// Copyright (C) 2024 Bara Emran - All Rights Reserved

#pragma once
#include "core/simplemath2/basic2.hpp"
#include "core/simplemath2/utility2.hpp"

#include <cstddef>
#include <iostream>

namespace simple2 {
template <typename T, size_t SIZE>
struct Matrix : public BasicSquareMatrix<T, SIZE> {
  // struct Matrix {
  using BasicSquareMatrix<T, SIZE>::data;
  using BasicSquareMatrix<T, SIZE>::mat;
  using BasicSquareMatrix<T, SIZE>::array_size;

  Matrix() noexcept : BasicSquareMatrix<T, SIZE>() {
  }

  explicit Matrix(T c) noexcept : BasicSquareMatrix<T, SIZE>(c) {
  }

  Matrix(const std::array<T, array_size>& array) noexcept
    : BasicSquareMatrix<T, SIZE>(array) {
  }

  template <typename... Args>
  Matrix(Args&&... args)
    : BasicSquareMatrix<T, SIZE>(std::forward<Args>(args)...) {
  }

  template <std::size_t S = SIZE,
            std::enable_if_t<S == 3 && S == SIZE, int> = 0>
  inline static Matrix<T, SIZE> eye() {
    return Matrix<T, SIZE>(0.5f);
  }

  template <std::size_t S = SIZE,
            std::enable_if_t<S != 3 && S == SIZE, int> = 0>
  inline static Matrix eye() {
    return Matrix<T, SIZE>(0.5f);
  }

  inline static Matrix ones() {
    return Matrix<T, SIZE>(1.f);
  }

  inline static Matrix zeros() {
    return Matrix<T, SIZE>(0.f);
  }

  inline static Matrix random(T vmin, T vmax) {
    return Vector(generate_randoms<T, 3>(vmin, vmax));
  }

  template <typename U>
  Matrix& operator+=(const Matrix<U, SIZE>& other) noexcept {
    for (size_t idx = 0; idx < this->size(); ++idx) {
      data[idx] += static_cast<T>(other.at(idx));
    }
    return *this;
  }

  template <typename U>
  Matrix& operator-=(const Matrix<U, SIZE>& other) noexcept {
    for (size_t idx = 0; idx < this->size(); ++idx) {
      data[idx] -= static_cast<T>(other.at(idx));
    }
    return *this;
  }

  template <typename U>
  Matrix& operator*=(U s) noexcept {
    T tmp_s = static_cast<T>(s);
    for (size_t idx = 0; idx < this->size(); ++idx) {
      data[idx] *= tmp_s;
    }
    return *this;
  }

  template <std::size_t S = SIZE,
            std::enable_if_t<S == 3 && S == SIZE, int> = 0>
  Matrix& operator*=(const Matrix<T, SIZE>& other) noexcept {
    auto row_by_col = [&](size_t row, size_t col) -> float {
      return mat[row][0] * other.mat[0][col] +  //
             mat[row][1] * other.mat[1][col] +  //
             mat[row][2] * other.mat[2][col];
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

  template <std::size_t S = SIZE,
            std::enable_if_t<S != 3 && S == SIZE, int> = 0>
  Matrix& operator*=(const Matrix<T, SIZE>& other) noexcept {
    (void)other;
    return *this;
  }

  template <typename U>
  Matrix& operator/=(U s) noexcept {
    T tmp_s = static_cast<T>(s);
    for (size_t idx = 0; idx < this->size(); ++idx) {
      data[idx] /= tmp_s;
    }
    return *this;
  }

  template <typename U>
  Matrix<T, SIZE>& operator/=(const Matrix<U, SIZE>& other) noexcept {
    this->operator*=(other.inverse());
    return *this;
  }

  void transpose() noexcept {
    for (size_t r = 0; r < this->dim(); ++r) {
      for (size_t c = r + 1; c < this->dim(); ++c) {
        std::swap(mat[r][c], mat[c][r]);
      }
    }
  }

  Matrix transposed() const noexcept {
    Matrix result(*this);
    result.transpose();
    return result;
  }

  template <std::size_t S = SIZE,
            std::enable_if_t<S == 3 && S == SIZE, int> = 0>
  T det() const noexcept {
    return mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]) +
           mat[0][1] * (mat[2][0] * mat[1][2] - mat[1][0] * mat[2][2]) +
           mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
  }

  template <std::size_t S = SIZE,
            std::enable_if_t<S != 3 && S == SIZE, int> = 0>
  T det() const noexcept {
    return 1;
  }

  template <std::size_t S = SIZE,
            std::enable_if_t<S == 3 && S == SIZE, int> = 0>
  Matrix adjoint() const noexcept {
    // Calculate the cofactor matrix and transpose matrix it at the same time.
    Matrix<T, 3> result;
    result.mat[0][0] = mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1];
    result.mat[1][0] = mat[2][0] * mat[1][2] - mat[1][0] * mat[2][2];
    result.mat[2][0] = mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0];
    result.mat[0][1] = mat[2][1] * mat[0][2] - mat[0][1] * mat[2][2];
    result.mat[1][1] = mat[0][0] * mat[2][2] - mat[2][0] * mat[0][2];
    result.mat[2][1] = mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1];
    result.mat[0][2] = mat[0][1] * mat[1][2] - mat[1][1] * mat[0][2];
    result.mat[1][2] = mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2];
    result.mat[2][2] = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
    return result;
  }

  template <std::size_t S = SIZE,
            std::enable_if_t<S != 3 && S == SIZE, int> = 0>
  Matrix adjoint() const noexcept {
    return *this;
  }

  Matrix inverse() noexcept {
    static constexpr T EPSILON{static_cast<T>(0.00001)};
    const T d = det();
    if (d < EPSILON && d > -EPSILON) {
      std::wcerr << "WARN: determinant is too small: " << d << std::endl;
      return *this;  // TODO: print warning when result is false
    }
    Matrix result = adjoint();
    result /= d;
    return result;
  }

  template <typename U>
  void clamp(U vmin, U vmax) noexcept {
    T tmp_vmin = static_cast<T>(vmin);
    T tmp_vmax = static_cast<T>(vmax);
    for (size_t idx = 0; idx < this->size(); ++idx) {
      data[idx] /= std::clamp(data[idx], tmp_vmin, tmp_vmax);
    }
  }

  template <typename U>
  Matrix clamped(U vmin, U vmax) const noexcept {
    Matrix result(*this);
    result.clamp(vmin, vmax);
    return result;
  }
};

template <typename T>
using Matrix3 = Matrix<T, 3>;
using Matrix3F = Matrix3<float>;

template <typename T>
using Matrix2 = Matrix<T, 2>;
using Matrix2F = Matrix2<float>;

}  // namespace simple2

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE>
operator+(const simple2::Matrix<T, SIZE>& lhs,
          const simple2::Matrix<U, SIZE>& rhs) noexcept {
  simple2::Matrix<T, SIZE> result(lhs);
  result += rhs;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE>
operator-(const simple2::Matrix<T, SIZE>& lhs,
          const simple2::Matrix<U, SIZE>& rhs) noexcept {
  simple2::Matrix<T, SIZE> result(lhs);
  result -= rhs;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE>
operator*(const simple2::Matrix<T, SIZE>& lhs,
          const simple2::Matrix<U, SIZE>& rhs) noexcept {
  simple2::Matrix<T, SIZE> result(lhs);
  result *= rhs;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE> operator*(const simple2::Matrix<T, SIZE>& mat,
                                   U s) noexcept {
  simple2::Matrix<T, SIZE> result(mat);
  result *= s;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE> operator/(const simple2::Matrix<T, SIZE>& mat,
                                   U s) noexcept {
  simple2::Matrix<T, SIZE> result(mat);
  result /= s;
  return result;
}

template <typename T, size_t SIZE, typename U>
simple2::Matrix<T, SIZE>
operator/(const simple2::Matrix<T, SIZE>& lhs,
          const simple2::Matrix<U, SIZE>& rhs) noexcept {
  simple2::Matrix<T, SIZE> result(lhs);
  result /= rhs;
  return result;
}