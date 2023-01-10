#ifndef CORE_MATH_MATRIX_X_HPP_
#define CORE_MATH_MATRIX_X_HPP_

#include <eigen3/Eigen/Core>

#include "core/math/math.hpp"
using core::math::MATH_TYPE;

/**
 * @brief Class to represent a square matrix
 */
template <size_t SIZE>
class MatX {
 public:
  using MatType = Eigen::Matrix<MATH_TYPE, SIZE, SIZE>;

  inline MatX() : mat{MatType::Identity()} {
  }

  inline MatX(const MatType& matrix) : mat{matrix} {
  }

  inline MatType Matrix() const {
    return mat;
  }

  inline MatType& Matrix() {
    return mat;
  }

  inline MatX Inv() const {
    return MatType(mat.inverse().data());
  }

  inline void InvInPlace() {
    mat = Inv();
  }

  inline MatX T() const {
    return Mat3(mat.transpose().data());
  }

  inline void TransposeInPlace() {
    mat.transposeInPlace();
  }

  inline float Det() const {
    return mat.determinant();
  }

  inline float Norm() const {
    return mat.norm();
  }

  inline float Trace() const {
    return mat.trace();
  }

 protected:
  MatType mat;
};

#endif  // CORE_MATH_MATRIX_X_HPP_