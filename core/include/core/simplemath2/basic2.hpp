// Copyright (C) 2024 Bara Emran - All Rights Reserved

#pragma once
#include <cstddef>
#include <algorithm>
#include <array>

namespace simple2 {

template <typename T, size_t SIZE>
struct BasicVector {
  using iterator = T*;
  using const_iterator = const T*;
  static const size_t array_size{SIZE};
  T data[array_size] = {};

  BasicVector() {
  }

  BasicVector(T c) noexcept {
    std::fill_n(data, array_size, c);
  }

  BasicVector(std::array<T, SIZE>& array) noexcept {
    std::fill(array.begin(), array.end(), data);
  }

  inline T& at(size_t idx) noexcept {
    return data[idx];
  }

  inline T at(size_t idx) const noexcept {
    return data[idx];
  }

  constexpr inline size_t size() const noexcept {
    return array_size;
  }

  constexpr iterator begin() noexcept {
    return data;
  }

  constexpr const_iterator begin() const noexcept {
    return data;
  }

  constexpr iterator end() noexcept {
    return data + array_size;
  }

  constexpr const_iterator end() const noexcept {
    return data + array_size;
  }
};

template <typename T, size_t SIZE>
struct BasicSquareMatrix {
  using iterator = T*;
  using const_iterator = const T*;
  static constexpr size_t array_size{SIZE * SIZE};
  union {
    T mat[SIZE][SIZE];
    T data[array_size] = {};
  };

  BasicSquareMatrix() {
  }

  BasicSquareMatrix(T c) noexcept {
    std::fill_n(data, array_size, c);
  }

  BasicSquareMatrix(const std::array<T, array_size>& array) noexcept {
    std::copy(array.begin(), array.end(), data);
  }

  template <std::size_t S = SIZE,
            std::enable_if_t<S == 3 && S == SIZE, int> = 0>
  BasicSquareMatrix(T m00, T m01, T m02, T m10, T m11, T m12, T m20, T m21,
                    T m22) noexcept
    : data{m00, m01, m02, m10, m11, m12, m20, m21, m22} {
  }
  
  inline T& at(size_t row, size_t col) noexcept {
    return data[row * SIZE + col];
  }

  inline T at(size_t row, size_t col) const noexcept {
    return data[row * SIZE + col];
  }

  inline T& at(size_t idx) noexcept {
    return data[idx];
  }

  inline T at(size_t idx) const noexcept {
    return data[idx];
  }

  constexpr inline size_t size() const noexcept {
    return array_size;
  }

  constexpr inline size_t dim() const noexcept {
    return SIZE;
  }

  constexpr iterator begin() noexcept {
    return data;
  }

  constexpr const_iterator begin() const noexcept {
    return data;
  }

  constexpr iterator end() noexcept {
    return data + array_size;
  }

  constexpr const_iterator end() const noexcept {
    return data + array_size;
  }
};
}  // namespace simple2