#include "dcm.hpp"

namespace my {
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
  const float angle = std::acos((Trace() - 1.f) / 2.f);
  const Mat3 s = mat - T().Matrix();
  // Skew - symmetric matrix
  const Mat3 logR = angle * s / (2.f * std::sin(angle));
  return logR;
}

Mat3 DCM::Adjugate() const {
  return Det() * T().Matrix();
}
}  // namespace my
