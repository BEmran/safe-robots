#include "transformation.hpp"

#include <algorithm>  // clamp
#include <iostream>   // cerr

namespace my {
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

std::pair<float, Vec3> DCMToAxisAngle(const DCM& dcm) {
  float angle = std::acos((dcm.Trace() - 1) / 2.f);
  Vec3 axis = Vec3::Zero();
  if (angle == 0) {
    return {0, axis};
  }
  const auto mat = dcm.Matrix();
  axis.x() = mat(2, 1) - mat(1, 2);
  axis.y() = mat(0, 2) - mat(2, 0);
  axis.z() = mat(1, 0) - mat(0, 1);
  axis = axis / (2 * std::sin(angle));
  return {angle, axis};
}

DCM AxisAngleToDCM(const float angle, const Vec3 axis) {
  const Mat3 K = Skew(axis.normalized());
  const Mat3 dcm =
    Mat3::Identity() + std::sin(angle) * K + (1 - std::cos(angle)) * K * K;
  return dcm;
}

DCM FromEulerXYZ(const RPY rpy) {
  return Mat3(Eigen::AngleAxisf(rpy.roll, Vec3::UnitX()) *
              Eigen::AngleAxisf(rpy.pitch, Vec3::UnitY()) *
              Eigen::AngleAxisf(rpy.yaw, Vec3::UnitZ()));
}

DCM FromEulerZYX(const RPY rpy) {
  return Mat3(Eigen::AngleAxisf(rpy.yaw, Vec3::UnitZ()) *
              Eigen::AngleAxisf(rpy.pitch, Vec3::UnitY()) *
              Eigen::AngleAxisf(rpy.roll, Vec3::UnitX()));
}

DCM EulerToDCM(const RPY rpy, const EulerOrder order) {
  switch (order) {
    case EulerOrder::XYZ:
      return FromEulerXYZ(rpy);
    case EulerOrder::ZYX:
      return FromEulerZYX(rpy);
    default:
      std::cout << "Undefined EulerOrder" << std::endl;
      return DCM();
  }
}

RPY DCMToEulerXYZ(const DCM dcm) {
  const Mat3& mat{dcm.Matrix()};
  const float alpha = std::atan2(-mat(1, 2), mat(2, 2));
  const float beta =
    std::atan2(mat(0, 2), (std::sqrt(1 - mat(0, 2) * mat(0, 2))));
  const float gamma = std::atan2(-mat(0, 1), mat(0, 0));
  return RPY(alpha, beta, gamma);
}

RPY DCMToEulerZYX(const DCM dcm) {
  const Mat3& mat{dcm.Matrix()};
  const float alpha = std::atan2(mat(1, 0), mat(0, 0));
  const float beta =
    std::atan2(-mat(2, 0), (std::sqrt(1 - mat(2, 0) * mat(2, 0))));
  const float gamma = std::atan2(mat(2, 1), mat(2, 2));
  return RPY(gamma, beta, alpha);
}

RPY DCMToEuler(const DCM& dcm, const EulerOrder order) {
  switch (order) {
    case EulerOrder::XYZ:
      return DCMToEulerXYZ(dcm.Matrix());
    case EulerOrder::ZYX:
      return DCMToEulerZYX(dcm.Matrix());
    default:
      std::cerr << "Undefined EulerOrder" << std::endl;
      return RPY();
  }
}

Quaternion Shepperd2(const DCM dcm) {
  const Mat3& mat{dcm.Matrix()};
  const float trace = dcm.Trace();
  const Vec3 diag = mat.diagonal();

  std::array<float, 4> b{trace, diag(0), diag(1), diag(2)};
  const auto ptr = std::max_element(b.begin(), b.end());
  const auto idx = std::distance(b.begin(), ptr);

  float w{1.f}, x{0.f}, y{0.f}, z{0.f};
  float gain{1.f};
  std::cout << "idx: " << idx << std::endl;
  switch (idx) {
    case 0:
      w = 1.f + diag.sum();
      x = mat(1, 2) - mat(2, 1);
      y = mat(2, 0) - mat(0, 2);
      z = mat(0, 1) - mat(1, 0);
      gain = 2 * std::sqrt(w);
      break;
    case 1:
      w = mat(1, 2) - mat(2, 1);
      x = 1.f + diag(0) - diag(1) - diag(2);
      y = mat(1, 0) + mat(0, 1);
      z = mat(2, 0) + mat(0, 2);
      gain = 2.f * std::sqrt(x);
      break;
    case 2:
      w = mat(2, 0) - mat(0, 2);
      x = mat(1, 0) + mat(0, 1);
      y = 1.f - diag(0) + diag(1) - diag(2);
      z = mat(2, 1) + mat(1, 2);
      gain = 2.f * std::sqrt(y);
      break;
    case 3:
      w = mat(0, 1) - mat(1, 0);
      x = mat(2, 0) + mat(0, 2);
      y = mat(2, 1) + mat(1, 2);
      z = 1.f - diag(0) - diag(1) + diag(2);
      gain = 2.f * std::sqrt(z);
      break;
    default:
      std::cerr << "Undefined index" << std::endl;
  }
  return Quaternion(w / gain, x / gain, y / gain, z / gain);
}

Quaternion Shepperd(const DCM dcm) {
  const Mat3& mat{dcm.Matrix()};
  const Vec3 diag = mat.diagonal();
  std::array<float, 4> b{dcm.Trace(), diag[0], diag[1], diag[2]};
  const auto ptr = std::max_element(b.begin(), b.end());
  const auto idx = std::distance(b.begin(), ptr);
  std::array<float, 4> q{1.f, 0.f, 0.f, 0.f};
  switch (idx) {
    case 0:
      q[0] = 1.f + dcm.Trace();
      q[1] = mat(1, 2) - mat(2, 1);
      q[2] = mat(2, 0) - mat(0, 2);
      q[3] = mat(0, 1) - mat(1, 0);
      break;
    case 1:
      q[0] = mat(1, 2) - mat(2, 1);
      q[1] = 1.f + diag[0] - diag[1] - diag[2];
      q[2] = mat(1, 0) + mat(0, 1);
      q[3] = mat(2, 0) + mat(0, 2);
      break;
    case 2:
      q[0] = mat(2, 0) - mat(0, 2);
      q[1] = mat(1, 0) + mat(0, 1);
      q[2] = 1.f - diag[0] + diag[1] - diag[2];
      q[3] = mat(2, 1) + mat(1, 2);
      break;
    case 3:
      q[0] = mat(0, 1) - mat(1, 0);
      q[1] = mat(2, 0) + mat(0, 2);
      q[2] = mat(2, 1) + mat(1, 2);
      q[3] = 1.f - diag[0] - diag[1] + diag[2];
      break;
    default:
      std::cerr << "wrong index" << std::endl;
      return Quaternion();
  }
  const float gain = 0.5f / std::sqrt(q[static_cast<size_t>(idx)]);
  Quaternion quat(q[0] / gain, q[1] / gain, q[2] / gain, q[3] / gain);
  quat.Normalize();
  // TODO: use the conjugate quaternion. not sure if it is correct but the unit
  // test returns the same quaternion used to generate the dcm matrix!!!
  return quat.Conjugate();
}

Quaternion Sarabandi(const DCM dcm) {
  const float eta = 1;
  const Mat3& mat{dcm.Matrix()};

  //  Compute qw
  const float dw = mat(0, 0) + mat(1, 1) + mat(2, 2);
  float qw{1.0};
  if (dw > eta) {
    qw = 0.5f * std::sqrt(1.f + dw);
  } else {
    const float nom = Square(mat(2, 1) - mat(1, 2)) +
                      Square(mat(0, 2) - mat(2, 0)) +
                      Square(mat(1, 0) - mat(0, 1));
    const float denom = 3.f - dw;
    qw = 0.5f * std::sqrt(nom / denom);
  }

  //  Compute qx
  const float dx = mat(0, 0) - mat(1, 1) - mat(2, 2);
  float qx{0.f};
  if (dx > eta) {
    qx = 0.5f * std::sqrt(1.f + dx);
  } else {
    const float nom = Square(mat(2, 1) - mat(1, 2)) +
                      Square(mat(0, 1) + mat(1, 0)) +
                      Square(mat(2, 0) + mat(0, 2));
    const float denom = 3.f - dx;
    qx = 0.5f * std::sqrt(nom / denom);
  }

  //  Compute qy
  const float dy = -mat(0, 0) + mat(1, 1) - mat(2, 2);
  float qy{0.f};
  if (dy > eta) {
    qy = 0.5f * std::sqrt(1.f + dy);
  } else {
    const float nom = Square(mat(0, 2) - mat(2, 0)) +
                      Square(mat(0, 1) + mat(1, 0)) +
                      Square(mat(1, 2) + mat(2, 1));
    const float denom = 3.f - dy;
    qy = 0.5f * std::sqrt(nom / denom);
  }

  // Compute qz
  const float dz = -mat(0, 0) - mat(1, 1) + mat(2, 2);
  float qz{0.f};
  if (dz > eta) {
    qz = 0.5f * std::sqrt(1.f + dz);
  } else {
    const float nom = Square(mat(1, 0) - mat(0, 1)) +
                      Square(mat(2, 0) + mat(0, 2)) +
                      Square(mat(1, 2) + mat(2, 1));
    const float denom = 3.f - dz;
    qz = 0.5f * std::sqrt(nom / denom);
  }
  return Quaternion(qw, qx, qy, qz);
}

Quaternion Chiaverini(const DCM dcm) {
  const Mat3& mat{dcm.Matrix()};
  const float qx_tmp = std::clamp(mat(0, 0) - mat(1, 1) - mat(2, 2), -1.f, 1.f);
  const float qy_tmp = std::clamp(mat(1, 1) - mat(2, 2) - mat(0, 0), -1.f, 1.f);
  const float qz_tmp = std::clamp(mat(2, 2) - mat(0, 0) - mat(1, 1), -1.f, 1.f);
  const float qw = 0.5f * std::sqrt(dcm.Trace() + 1.f);
  const float qx = 0.5f * Sign(mat(2, 1) - mat(1, 2)) * std::sqrt(qx_tmp + 1.f);
  const float qy = 0.5f * Sign(mat(0, 2) - mat(2, 0)) * std::sqrt(qy_tmp + 1.f);
  const float qz = 0.5f * Sign(mat(1, 0) - mat(0, 1)) * std::sqrt(qz_tmp + 1.f);
  return Quaternion(qw, qx, qy, qz).Normalized();
}

Quaternion DCMToQuaternion(const DCM& dcm, const QuaternionMethod method) {
  switch (method) {
    case QuaternionMethod::SHEPPERD:
      return Shepperd(dcm.Matrix());
    case QuaternionMethod::SARABANDI:
      return Sarabandi(dcm.Matrix());
    case QuaternionMethod::CHIAVERINI:
      return Chiaverini(dcm.Matrix());
    default:
      std::cerr << "Undefined Quaternion Method" << std::endl;
      return Quaternion();
  }
}

DCM QuatToDCM(const Quaternion quat) {
  const Quaternion q = quat.Normalized();
  Mat3 R = Mat3::Identity();
  R(0, 0) = 1.f - 2.f * (q.Y() * q.Y() + q.Z() * q.Z());
  R(0, 1) = 2.f * (q.X() * q.Y() - q.W() * q.Z());
  R(0, 2) = 2.f * (q.X() * q.Z() + q.W() * q.Y());
  R(1, 0) = 2.f * (q.X() * q.Y() + q.W() * q.Z());
  R(1, 1) = 1.f - 2.f * (q.X() * q.X() + q.Z() * q.Z());
  R(1, 2) = 2.f * (q.Y() * q.Z() - q.W() * q.X());
  R(2, 0) = 2.f * (q.X() * q.Z() - q.W() * q.Y());
  R(2, 1) = 2.f * (q.W() * q.X() + q.Y() * q.Z());
  R(2, 2) = 1.f - 2.f * (q.X() * q.X() + q.Y() * q.Y());
  return R;
}
}  // namespace my