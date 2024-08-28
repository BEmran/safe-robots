#pragma once

#include <memory>
#include "my_array.hpp"
#include "my_vector.hpp"
#include <array>
#include <cassert>
#include <ostream>

template <typename Tp, std::size_t ROWS, std::size_t COLS>
class Matrix : public Array<Tp, ROWS * COLS> {
 public:
  using BaseType = Array<Tp, ROWS * COLS>;
  Matrix() = default;

  Matrix(const BaseType& other) : BaseType(other) {
  }

  Matrix(BaseType&& other) : BaseType(std::move(other)) {
  }

  static Matrix<Tp, ROWS, COLS> zeros() {
    return std::move(BaseType(0.0));
  }

  static Matrix<Tp, ROWS, COLS> ones() {
    return std::move(BaseType(1.0));
  }

  static Matrix<Tp, ROWS, COLS> random() {
    BaseType arr(0.0);
    std::generate(arr.begin(), arr.end(), generate_random);
    return std::move(arr);
  }

  constexpr Tp& at(size_t row, size_t col) {
    return this->at(row * COLS + col);
  }

  constexpr const Tp& at(size_t row, size_t col) const {
    return this->at(row * COLS + col);
  }

  constexpr Tp& at(size_t idx) {
    return BaseType::at(idx);
  }

  constexpr const Tp& at(size_t idx) const {
    return BaseType::at(idx);
  }

  constexpr size_t rows() const {
    return ROWS;
  }

  constexpr size_t cols() const {
    return COLS;
  }

  Vector<Tp, ROWS> col(size_t idx) const {
    assert(idx < COLS);
    Vector<Tp, ROWS> res;
    size_t iter = 0;
    for (size_t i = idx; i < this->size(); i += COLS) {
      res[iter++] = this->at(i);
    }
    return res;
  }

  Vector<Tp, COLS> row(size_t idx) const {
    assert(idx < ROWS);
    Vector<Tp, COLS> res;
    std::copy_n(this->cbegin() + idx * COLS, COLS, res.begin());
    return res;
  }

  std::string print() const {
    std::stringstream ss;
    ss << "[";
    for (size_t r = 0; r < ROWS; ++r) {
      std::copy_n(this->cbegin() + r * COLS, COLS,
                  std::ostream_iterator<Tp>(ss, ", "));
      if (r < ROWS - 1) {
        ss << "\n";
      }
    }
    ss << "]";
    return ss.str();
  }

  Matrix<Tp, ROWS, COLS> dot(const Matrix<Tp, ROWS, COLS>& other) const {
    Matrix<Tp, ROWS, COLS> result;
    for (auto i = 0; i < this->size(); i++) {
      result.at(i) = this->at(i) * other.at(i);
    }
    return std::move(result);
  }

  template <size_t ROWS2, size_t COLS2>
  Matrix<Tp, ROWS, COLS2> mul(const Matrix<Tp, ROWS2, COLS2>& other) const {
    if (COLS2 != ROWS2) {
      std::__throw_out_of_range_fmt(__N("Matrix::mul: left hand side matrix "
                                        "has size of %zux%zu."
                                        "It should have %zu number of rows"),
                                    ROWS2, COLS2, COLS);
    }
    // Matrix<Tp, ROWS, COLS2> result;
    // for (auto r = 0; r < ROWS; r++) {
    //   for (auto c = 0; c < COLS2; c++) {
    //     result.at(r, c) = this->row(r).mul(other.col(c));
    //   }
    // }
    // return std::move(result);

    Matrix<Tp, ROWS, COLS2> result;
    for (auto r = 0; r < ROWS; ++r) {
      for (auto c = 0; c < COLS2; ++c) {
        Tp sum = 0.0;
        for (auto i = 0; i < ROWS2; ++i) {
          sum += this->at(r, i) * other.at(i, c);
        }
        result.at(r, c) = sum;
      }
    }
    return std::move(result);
  }

  template <size_t SIZE>
  Vector<Tp, ROWS> mul(const Vector<Tp, SIZE>& other) const {
    if (COLS != SIZE) {
      std::__throw_out_of_range_fmt(__N("Matrix::mul: left hand side vector "
                                        "has size of %zu."
                                        "It should have %zu number of "
                                        "elements"),
                                    SIZE, COLS);
    }
    Vector<Tp, ROWS> result;
    for (auto r = 0; r < ROWS; r++) {
      result.at(r) = this->row(r).mul(other);
    }
    return std::move(result);
  }
};

template <typename Tp, std::size_t ROWS, std::size_t COLS>
Matrix<Tp, ROWS, COLS> dot(const Matrix<Tp, ROWS, COLS>& lhs,
                           const Matrix<Tp, ROWS, COLS>& rhs) {
  return lhs.dot(rhs);
}

template <typename Tp, std::size_t ROWS_L, std::size_t COLS_L,
          std::size_t ROWS_R, std::size_t COLS_R>
Matrix<Tp, ROWS_L, COLS_R> mul(const Matrix<Tp, ROWS_L, COLS_L>& lhs,
                               const Matrix<Tp, ROWS_R, COLS_R>& rhs) {
  return lhs.mul(rhs);
}

template <typename Tp, std::size_t ROWS, std::size_t COLS, std::size_t SIZE>
Vector<Tp, ROWS> mul(const Matrix<Tp, ROWS, COLS>& lhs,
                     const Vector<Tp, SIZE>& rhs) {
  return lhs.mul(rhs);
}