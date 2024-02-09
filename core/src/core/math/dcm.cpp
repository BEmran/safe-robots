#include "core/math/dcm.hpp"

#include <stdexcept>

namespace core::math {
DCM::DCM() : mat{Mat3::Identity()} {
}

DCM::DCM(const Mat3& matrix) : mat{matrix} {
}

Mat3 DCM::Matrix() const {
  return mat;
}

Mat3& DCM::Matrix() {
  return mat;
}

DCM DCM::Inv() const {
  return Mat3(mat.transpose().data());
}

void DCM::InvInPlace() {
  mat.transposeInPlace();
}

DCM DCM::T() const {
  return Mat3(mat.transpose().data());
}

void DCM::TransposeInPlace() {
  mat.transposeInPlace();
}

float DCM::Det() const {
  return mat.determinant();
}

float DCM::Norm() const {
  return mat.norm();
}

float DCM::Trace() const {
  return mat.trace();
}

Mat3 DCM::Log() const {
  const float angle = std::acos((Trace() - 1.F) / 2.F);
  const Mat3 s = mat - T().Matrix();
  // Skew - symmetric matrix
  const Mat3 logR = angle * s / (2.F * std::sin(angle));
  return logR;
}

Mat3 DCM::Adjugate() const {
  return Det() * T().Matrix();
}

MATH_TYPE DCM::operator[](const size_t idx) const {
  if (idx > 8) {
    throw std::out_of_range("expected index value in range of [0, 9)");
  }
  return mat(static_cast<Eigen::Index>(idx));
}

MATH_TYPE& DCM::operator[](const size_t idx) {
  if (idx > 8) {
    throw std::out_of_range("expected index value in range of [0, 9)");
  }
  return mat(static_cast<Eigen::Index>(idx));
}

MATH_TYPE DCM::At(size_t idx) const {
  return this->operator[](idx);
}

MATH_TYPE& DCM::At(size_t idx) {
  return this->operator[](idx);
}

MATH_TYPE DCM::At(size_t row, size_t col) const {
  if (row > 3 || col > 3) {
    throw std::out_of_range("expected row and col values in range of [0, 3)");
  }
  return this->operator[](row * 3 + col);
}

MATH_TYPE& DCM::At(size_t row, size_t col) {
  if (row > 3 || col > 3) {
    throw std::out_of_range("expected row and col values in range of [0, 3)");
  }
  return this->operator[](row * 3 + col);
}
}  // namespace core::math
