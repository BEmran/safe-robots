#ifndef CORE_MATH_DCM_HPP_
#define CORE_MATH_DCM_HPP_

#include <algorithm>  // clamp

#include "core/utils/math.hpp"
using core::utils::Mat3;
using core::utils::Quat;
using core::utils::RPY;
using core::utils::Vec3;

enum class EulerOrder { XYZ, ZYX };
enum class QuaternionMethod { SHEPPERD, SARABANDI, CHIAVERINI };

bool AssertSO3(const Mat3 dcm) {
  const float eps = 0.0001f;
  const bool det_of_1 = std::abs(dcm.determinant() - 1) < eps;
  const bool orthogonal = std::abs((dcm * dcm.transpose()).trace() - 3) < eps;
  return det_of_1 && orthogonal;
}

/**
 * @brief Construct a skew-symmetric matrix, a square matrix whose transpose
 * equals its negative
 *
 * @param vec vector to be used
 * @return Mat3 skew skew-symmetric matrix
 */
Mat3 Skew(const Vec3 vec) {
  Mat3 mat = Mat3::Zero();
  mat(2, 1) = +vec.x();
  mat(1, 2) = -vec.x();
  mat(0, 2) = +vec.y();
  mat(2, 0) = -vec.y();
  mat(1, 0) = +vec.z();
  mat(0, 1) = -vec.z();
  return mat;
}

/**
 * @brief Construct a DCM from quaternion vector
 *
 * @param quat quaternion vector
 * @return Mat3 rotation matrix
 */
Mat3 QuatToDCM(const Quat quat) {
  const Quat q = quat.normalized();
  Mat3 R = Mat3::Identity();
  R(0, 0) = 1.f - 2.f * (q.y() * q.y() + q.z() * q.z());
  R(0, 1) = 2.f * (q.x() * q.y() - q.w() * q.z());
  R(0, 2) = 2.f * (q.x() * q.z() + q.w() * q.y());
  R(1, 0) = 2.f * (q.x() * q.y() + q.w() * q.z());
  R(1, 1) = 1.f - 2.f * (q.x() * q.x() + q.z() * q.z());
  R(1, 2) = 2.f * (q.y() * q.z() - q.w() * q.x());
  R(2, 0) = 2.f * (q.x() * q.z() - q.w() * q.y());
  R(2, 1) = 2.f * (q.w() * q.x() + q.y() * q.z());
  R(2, 2) = 1.f - 2.f * (q.x() * q.x() + q.y() * q.y());
  return R;
}

/**
 * @brief Construct a DCM from axis-angle representation. Use
 * Rodrigue's formula to obtain the DCM from the axis-angle representation.
 *
 * @return Mat3 dcm matrix
 */
Mat3 AxisAngleToDCM(const float angle, const Vec3 axis) {
  const Mat3 K = Skew(axis.normalized());
  return Mat3::Identity() + std::sin(angle) * K + (1 - std::cos(angle)) * K * K;
}

Mat3 FromEulerXYZ(const RPY rpy) {
  return Mat3(Eigen::AngleAxisf(rpy.roll, Vec3::UnitX()) *
              Eigen::AngleAxisf(rpy.pitch, Vec3::UnitY()) *
              Eigen::AngleAxisf(rpy.yaw, Vec3::UnitZ()));
}

Mat3 FromEulerZYX(const RPY rpy) {
  return Mat3(Eigen::AngleAxisf(rpy.yaw, Vec3::UnitZ()) *
              Eigen::AngleAxisf(rpy.pitch, Vec3::UnitY()) *
              Eigen::AngleAxisf(rpy.roll, Vec3::UnitX()));
}

Mat3 EulerToDCM(const RPY rpy, const EulerOrder order) {
  switch (order) {
    case EulerOrder::XYZ:
      return FromEulerXYZ(rpy);
    case EulerOrder::ZYX:
      return FromEulerZYX(rpy);
    default:
      std::cout << "Undefined EulerOrder" << std::endl;
      return Mat3::Identity();
  }
}

/**
 * @brief Roll-Pitch-Yaw Angles from DCM
 *
 * @return Vec3 a vector with [roll, pitch, yaw] angles
 */
RPY ToEulerXYZ(const Mat3 mat) {
  const float alpha = std::atan2(-mat(1, 2), mat(2, 2));
  const float beta =
    std::atan2(mat(0, 2), (std::sqrt(1 - mat(0, 2) * mat(0, 2))));
  const float gamma = std::atan2(-mat(0, 1), mat(0, 0));
  return RPY(alpha, beta, gamma);
}

RPY ToEulerZYX(const Mat3 mat) {
  const float alpha = std::atan2(mat(1, 0), mat(0, 0));
  const float beta =
    std::atan2(-mat(2, 0), (std::sqrt(1 - mat(2, 0) * mat(2, 0))));
  const float gamma = std::atan2(mat(2, 1), mat(2, 2));
  return RPY(gamma, beta, alpha);
}

template <typename T>
T Sign(const T val) {
  return static_cast<T>((T(0) < val) - (val < T(0)));
}

template <typename T>
T Square(const T val) {
  return val * val;
}

/**
 * @brief Quaternion from a Direction Cosine Matrix with Shepperd's method
 *
 * @param mat Direction Cosine Matrix
 * @return Quat quaternion
 */
Quat Shepperd2(const Mat3 dcm) {
  const Vec3 diag = dcm.diagonal();
  const float trace = dcm.trace();

  std::array<float, 4> b{trace, diag(0), diag(1), diag(2)};
  const auto ptr = std::max_element(b.begin(), b.end());
  const auto idx = std::distance(b.begin(), ptr);

  float w{1.f}, x{0.f}, y{0.f}, z{0.f};
  float gain{1.f};
  std::cout << "idx: " << idx << std::endl;
  switch (idx) {
    case 0:
      w = 1.f + diag.sum();
      x = dcm(1, 2) - dcm(2, 1);
      y = dcm(2, 0) - dcm(0, 2);
      z = dcm(0, 1) - dcm(1, 0);
      gain = 2 * std::sqrt(w);
      break;
    case 1:
      w = dcm(1, 2) - dcm(2, 1);
      x = 1.f + diag(0) - diag(1) - diag(2);
      y = dcm(1, 0) + dcm(0, 1);
      z = dcm(2, 0) + dcm(0, 2);
      gain = 2.f * std::sqrt(x);
      break;
    case 2:
      w = dcm(2, 0) - dcm(0, 2);
      x = dcm(1, 0) + dcm(0, 1);
      y = 1.f - diag(0) + diag(1) - diag(2);
      z = dcm(2, 1) + dcm(1, 2);
      gain = 2.f * std::sqrt(y);
      break;
    case 3:
      w = dcm(0, 1) - dcm(1, 0);
      x = dcm(2, 0) + dcm(0, 2);
      y = dcm(2, 1) + dcm(1, 2);
      z = 1.f - diag(0) - diag(1) + diag(2);
      gain = 2.f * std::sqrt(z);
      break;
    default:
      std::cerr << "Undefined index" << std::endl;
  }
  return Quat(w / gain, x / gain, y / gain, z / gain);
}

Quat Shepperd(const Mat3 dcm) {
  const Vec3 diag = dcm.diagonal();
  std::array<float, 4> b{dcm.trace(), diag[0], diag[1], diag[2]};
  const auto ptr = std::max_element(b.begin(), b.end());
  const auto idx = std::distance(b.begin(), ptr);
  std::array<float, 4> q{1.f, 0.f, 0.f, 0.f};
  switch (idx) {
    case 0:
      q[0] = 1.f + dcm.trace();
      q[1] = dcm(1, 2) - dcm(2, 1);
      q[2] = dcm(2, 0) - dcm(0, 2);
      q[3] = dcm(0, 1) - dcm(1, 0);
      break;
    case 1:
      q[0] = dcm(1, 2) - dcm(2, 1);
      q[1] = 1.f + diag[0] - diag[1] - diag[2];
      q[2] = dcm(1, 0) + dcm(0, 1);
      q[3] = dcm(2, 0) + dcm(0, 2);
      break;
    case 2:
      q[0] = dcm(2, 0) - dcm(0, 2);
      q[1] = dcm(1, 0) + dcm(0, 1);
      q[2] = 1.f - diag[0] + diag[1] - diag[2];
      q[3] = dcm(2, 1) + dcm(1, 2);
      break;
    case 3:
      q[0] = dcm(0, 1) - dcm(1, 0);
      q[1] = dcm(2, 0) + dcm(0, 2);
      q[2] = dcm(2, 1) + dcm(1, 2);
      q[3] = 1.f - diag[0] - diag[1] + diag[2];
      break;
    default:
      std::cerr << "wrong index" << std::endl;
      return Quat();
  }
  const float gain = 0.5f / std::sqrt(q[static_cast<size_t>(idx)]);
  Quat quat(q[0] / gain, q[1] / gain, q[2] / gain, q[3] / gain);
  quat.normalize();
  // TODO: use the conjugate quaternion. not sure if it is correct but the unit
  // test returns the same quaternion used to generate the dcm matrix!!!
  return quat.conjugate();
}
/**
 * @brief Quaternion from a Direction Cosine Matrix with Sarabandi's method.
 *
 * @param mat Direction Cosine Matrix
 * @return Quat quaternion
 */
Quat Sarabandi(const Mat3 dcm) {
  const float eta = 1;

  //  Compute qw
  const float dw = dcm(0, 0) + dcm(1, 1) + dcm(2, 2);
  float qw{1.0};
  if (dw > eta) {
    qw = 0.5f * std::sqrt(1.f + dw);
  } else {
    const float nom = Square(dcm(2, 1) - dcm(1, 2)) +
                      Square(dcm(0, 2) - dcm(2, 0)) +
                      Square(dcm(1, 0) - dcm(0, 1));
    const float denom = 3.f - dw;
    qw = 0.5f * std::sqrt(nom / denom);
  }

  //  Compute qx
  const float dx = dcm(0, 0) - dcm(1, 1) - dcm(2, 2);
  float qx{0.f};
  if (dx > eta) {
    qx = 0.5f * std::sqrt(1.f + dx);
  } else {
    const float nom = Square(dcm(2, 1) - dcm(1, 2)) +
                      Square(dcm(0, 1) + dcm(1, 0)) +
                      Square(dcm(2, 0) + dcm(0, 2));
    const float denom = 3.f - dx;
    qx = 0.5f * std::sqrt(nom / denom);
  }

  //  Compute qy
  const float dy = -dcm(0, 0) + dcm(1, 1) - dcm(2, 2);
  float qy{0.f};
  if (dy > eta) {
    qy = 0.5f * std::sqrt(1.f + dy);
  } else {
    const float nom = Square(dcm(0, 2) - dcm(2, 0)) +
                      Square(dcm(0, 1) + dcm(1, 0)) +
                      Square(dcm(1, 2) + dcm(2, 1));
    const float denom = 3.f - dy;
    qy = 0.5f * std::sqrt(nom / denom);
  }

  // Compute qz
  const float dz = -dcm(0, 0) - dcm(1, 1) + dcm(2, 2);
  float qz{0.f};
  if (dz > eta) {
    qz = 0.5f * std::sqrt(1.f + dz);
  } else {
    const float nom = Square(dcm(1, 0) - dcm(0, 1)) +
                      Square(dcm(2, 0) + dcm(0, 2)) +
                      Square(dcm(1, 2) + dcm(2, 1));
    const float denom = 3.f - dz;
    qz = 0.5f * std::sqrt(nom / denom);
  }
  return Quat(qw, qx, qy, qz);
}

/**
 * @brief Quaternion from a Direction Cosine Matrix with Chiaverini's algebraic
 * method.
 *
 * @param mat Direction Cosine Matrix
 * @return Quat quaternion
 */
Quat Chiaverini(const Mat3 dcm) {
  const float qx_tmp = std::clamp(dcm(0, 0) - dcm(1, 1) - dcm(2, 2), -1.f, 1.f);
  const float qy_tmp = std::clamp(dcm(1, 1) - dcm(2, 2) - dcm(0, 0), -1.f, 1.f);
  const float qz_tmp = std::clamp(dcm(2, 2) - dcm(0, 0) - dcm(1, 1), -1.f, 1.f);
  const float qw = 0.5f * std::sqrt(dcm.trace() + 1.f);
  const float qx = 0.5f * Sign(dcm(2, 1) - dcm(1, 2)) * std::sqrt(qx_tmp + 1.f);
  const float qy = 0.5f * Sign(dcm(0, 2) - dcm(2, 0)) * std::sqrt(qy_tmp + 1.f);
  const float qz = 0.5f * Sign(dcm(1, 0) - dcm(0, 1)) * std::sqrt(qz_tmp + 1.f);
  return Quat(qw, qx, qy, qz).normalized();
}

/**
 * @brief Class to represent a Direction Cosine Rotation.It is built from a 3x3
 * array, but it can also be built from 3 - dimensional vectors representing the
 * *roll - pitch - yaw angles, a quaternion, or an axis - angle pair
 * representation.
 */
class DCM {
 public:
  DCM() : mat{Mat3::Identity()} {
  }

  DCM(const Mat3& matrix) : mat{matrix} {
  }

  DCM(const Quat& quat) : mat{QuatToDCM(quat)} {
  }

  DCM(const float angle, const Vec3 axis) : mat{AxisAngleToDCM(angle, axis)} {
  }

  DCM(const RPY rpy, const EulerOrder order) : mat{EulerToDCM(rpy, order)} {
  }

  Mat3 Matrix() const {
    return mat;
  }

  Mat3& Matrix() {
    return mat;
  }

  DCM Inv() const {
    return Mat3(mat.transpose().data());
  }

  void InvInPlace() {
    mat.transposeInPlace();
  }

  DCM T() const {
    return Mat3(mat.transpose().data());
  }

  void TransposeInPlace() {
    mat.transposeInPlace();
  }

  float Det() const {
    return mat.determinant();
  }

  float Norm() const {
    return mat.norm();
  }

  float Trace() const {
    return mat.trace();
  }

  /**
   * @brief Logarithm of DCM. The logarithmic map is defined as the inverse of
   * the exponential map. It corresponds to the logarithm given by the Rodrigues
   * rotation formula.
   *
   * @return Mat3 Logarithm of DCM
   */
  Mat3 Log() const {
    const float angle = std::acos((Trace() - 1.f) / 2.f);
    const Mat3 s = mat - T().Matrix();
    // Skew - symmetric matrix
    const Mat3 logR = angle * s / (2.f * std::sin(angle));
    return logR;
  }

  /**
   * @brief Return the adjugate of the DCM
   *
   * @return Mat3 adjugate matrix of DCM
   */
  Mat3 Adjugate() const {
    return Det() * T().Matrix();
  }

  /**
   * @brief  Return axis-angle representation of the DCM.
   * @details The axis-angle representation is not unique
   *
   * @return std::pair<float, Vec3> Angle of rotation [rad], Axis of rotation.
   */
  std::pair<float, Vec3> ToAxisAngle() const {
    float angle = std::acos((Trace() - 1) / 2.f);
    Vec3 axis = Vec3::Zero();
    if (angle == 0) {
      return {0, axis};
    }
    axis.x() = mat(2, 1) - mat(1, 2);
    axis.y() = mat(0, 2) - mat(2, 0);
    axis.z() = mat(1, 0) - mat(0, 1);
    axis = axis / (2 * std::sin(angle));
    return {angle, axis};
  }

  RPY ToEuler(const EulerOrder order) const {
    switch (order) {
      case EulerOrder::XYZ:
        return ToEulerXYZ(mat);
      case EulerOrder::ZYX:
        return ToEulerZYX(mat);
      default:
        std::cout << "Undefined EulerOrder" << std::endl;
        return RPY();
    }
  }

  Quat ToQuaternion(
    const QuaternionMethod method = QuaternionMethod::SHEPPERD) const {
    switch (method) {
      case QuaternionMethod::SHEPPERD:
        return Shepperd(mat);
      case QuaternionMethod::SARABANDI:
        return Sarabandi(mat);
      case QuaternionMethod::CHIAVERINI:
        return Chiaverini(mat);
      default:
        std::cout << "Undefined Quaternion Method" << std::endl;
        return Quat::Identity();
    }
  }

 private:
  Mat3 mat;
};

#endif  // CORE_MATH_DCM_HPP_