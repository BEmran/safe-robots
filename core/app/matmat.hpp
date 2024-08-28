#include <iostream>
#include <ostream>
#include <array>
#include <chrono>
#include <functional>

namespace matmat {

template <size_t ROWS, size_t COLS>
struct Matrix {
  std::array<float, ROWS * COLS> data;
  Matrix() {
    if (DEBUG) {
      std::cout << "default Constructor Matrix [" << ROWS * COLS
                << "]: " << this << std::endl;
    }
  }

  Matrix(const float& c) noexcept {
    if (DEBUG) {
      std::cout << "Constructor Matrix(" << c << ") [" << ROWS * COLS
                << "]: " << this << std::endl;
    }
    data.fill(c);
  }

  Matrix(const std::array<float, ROWS * COLS>& arr) noexcept {
    std::copy(arr.begin(), arr.end(), data.begin());
    if (DEBUG) {
      std::cout << "Constructor Matrix array [" << ROWS * COLS << "]: " << this
                << std::endl;
    }
  }

  Matrix(Matrix&& other) : data{std::move(other.data)} {
    if (DEBUG) {
      std::cout << "Constructor Matrix&& [" << ROWS * COLS << "]: " << this
                << std::endl;
    }
  }

  Matrix(const Matrix& other) : data{other.data} {
    if (DEBUG) {
      std::cout << "Constructor const Matrix& [" << ROWS * COLS << "]: " << this
                << std::endl;
    }
  }

  Matrix operator=(Matrix&& other) {
    data = std::move(other.data);
    if (DEBUG) {
      std::cout << "Constructor operator= Matrix&& [" << ROWS * COLS
                << "]: " << this << std::endl;
    }
  }

  Matrix operator=(const Matrix& other) {
    std::copy(other.data.begin(), other.data.end(), data.begin());
    if (DEBUG) {
      std::cout << "Constructor operator= const Matrix& [" << ROWS * COLS
                << "]: " << this << std::endl;
    }
  }

  ~Matrix() {
    if (DEBUG) {
      std::cout << "Destructor Matrix [" << ROWS * COLS << "] " << this
                << std::endl;
    }
  }

  inline constexpr size_t to_idx(size_t row, size_t col) const noexcept {
    return row * cols() + col;
  }

  constexpr float operator[](size_t idx) const noexcept {
    return data[idx];
  }

  inline constexpr float& operator[](size_t idx) noexcept {
    return data[idx];
  }

  inline constexpr float operator()(size_t idx) const noexcept {
    return data[idx];
  }

  inline constexpr float& operator()(size_t idx) noexcept {
    return data[idx];
  }

  inline constexpr float operator()(size_t row, size_t col) const noexcept {
    return data[to_idx(row, col)];
  }

  inline constexpr float& operator()(size_t row, size_t col) noexcept {
    return data[to_idx(row, col)];
  }

  inline constexpr size_t rows() const noexcept {
    return ROWS;
  }

  inline constexpr size_t cols() const noexcept {
    return COLS;
  }

  // std::array<float*, ROWS> row(size_t row) {
  //   std::array<float*, ROWS> result;
  //   for (size_t col = 0; col < cols(); ++col) {
  //     result[col] = &this->operator()(row, col);
  //   }
  //   return result;
  // }

  // std::array<float, COLS> row(size_t row) const{
  //   std::array<float, 1> result;
  //   auto start = data.begin() + row * cols(); 
  //   std::copy_n(start , cols(), result);
  //   return result;
  // }

  // std::array<float*, ROWS> col(size_t col) {
  //   std::array<float*, ROWS> result;
  //   for (size_t row = 0; row < rows(); ++row) {
  //     result[row] = &this->operator()(row, col);
  //   }
  //   return result;
  // }

  // std::array<float, ROWS> col(size_t col) const{
  //   std::array<float, ROWS> result;
  //   for (size_t row = 0; row < rows(); ++row) {
  //     result[row] = this->operator()(row, col);
  //   }
  //   return result;
  // }

  float multiply_row_col(size_t row, float col) {
    float result{0.f};
    if (rows() != cols()) {
      return result;
    }
    for (size_t i=0; i < rows(); ++i){
      result += data(row, i) + data(i, col);
    }
    return result;
  }

  // Matrix<ROWS, COLS> operator*(const Matrix<ROWS, COLS>& other) const {
  //   Matrix<ROWS, COLS> result(*this);

  //   if (DEBUG) {
  //     std::cout << "operator* [" << ROWS * COLS << "] " << this << std::endl;
  //   }
  //   return result;
  // }

  // void operator*=(const Matrix<ROWS, COLS>& other) {
  //   if (rows() != cols()) {
  //     return ;
  //   }
  //   Matrix<ROWS, COLS> tmp(*this);
  //   for (size_t r=0; r < rows(); ++r){
  //     for (size_t c=0; c < cols(); ++c){
  //       data[r * COLS + c] = tmp.multiply_row_col(r, c);
  //     }
  //   }

  //   if (DEBUG) {
  //     std::cout << "operator*= [" << ROWS * COLS << "] " << this <<
  //     std::endl;
  //   }
  // }
};
}  // namespace matmat

template <size_t ROWS, size_t COLS>
std::ostream& operator<<(std::ostream& os,
                         const matmat::Matrix<ROWS, COLS>& mat) {
  os << "[";
  for (size_t r = 0; r < ROWS; ++r) {
    os << "[";
    for (size_t c = 0; c < COLS; ++c) {
      os << mat(r, c);
      if (c < COLS - 1) {
        os << ", ";
      } else {
        os << "]";
      }
    }
    if (r < ROWS - 1) {
      os << "\n ";
    }
  }
  os << "]";
  return os;
}