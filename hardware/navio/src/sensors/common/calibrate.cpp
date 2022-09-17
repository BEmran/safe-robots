// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "sensors/common/calibrate.hpp"

#include <string>

#include "navio/hardware_utils.hpp"

namespace sensors::common::calibrate {
constexpr size_t kNumSamples = 1000;
constexpr auto ShortDelay = 5;
constexpr auto kEscKey = 'q';
constexpr auto kEpsilon = 0.025F;
constexpr const char* msg_request = " and press enter or q to quit .....";
constexpr const char* kFaceMsg[] = {"face up",   "right side", "left side",
                                    "nose down", "nose up",    "face down"};
bool GetUserApproval(const char* msg) {
  std::cout << msg << msg_request;
  const auto ch = getchar();
  if (ch == kEscKey) {
    std::cout << "User canceled the process" << std::endl;
    return false;
  }
  return true;
}

bool ExpectNear(const utils::Vec3& v1, const utils::Vec3& v2) {
  const auto error = v1 - v2;
  const auto norm = error.norm();
  if (norm > kEpsilon) {
    std::cout << "error vector = " << error.transpose() << " norm = " << norm
              << std::endl;
    return false;
  }
  return true;
}

utils::Vec3 GetAverage(const ReadFunc& cb) {
  utils::Vec3 bias = utils::Vec3::Zero();
  std::cout << "collecting data";
  constexpr auto packet_size = 10;
  for (size_t i = 0; i < kNumSamples; i++) {
    const auto data = cb();
    bias += data;
    navio::hardware_utils::Delay(ShortDelay);
    if (i % packet_size == 0) {
      std::cout << "." << std::flush;
    }
  }
  std::cout << std::endl;
  return bias / kNumSamples;
}

SensorSpecs<3> CalibrateAccelerometer(const ReadFunc& cb,
                                      const SensorSpecs<3>& spec) {
  std::cout << "Process of calibrating accelerometer" << std::endl;
  Eigen::Matrix<utils::MATH_TYPE, 6, 3> y;  // NOLINT
  y.setZero();
  y(0, 2) = +1.F;                           // NOLINT face up, z+
  y(1, 0) = -1.F;                           // NOLINT right side, x-
  y(2, 0) = +1.F;                           // NOLINT left side, x+
  y(3, 1) = -1.F;                           // NOLINT nose down, y-
  y(4, 1) = +1.F;                           // NOLINT nose up, y+
  y(5, 2) = -1.F;                           // NOLINT face down, z-
  Eigen::Matrix<utils::MATH_TYPE, 6, 4> x;  // NOLINT
  x.setZero();
  Eigen::Matrix<utils::MATH_TYPE, 4, 3> miss;  // NOLINT
  miss.setIdentity();
  for (Eigen::Index idx = 0; idx < y.rows(); idx++) {
    if (!GetUserApproval(kFaceMsg[idx])) {
      return spec;
    }
    auto xn = GetAverage(cb) / spec.sensitivity;

    std::cout << "xn[" << idx << "] = " << xn.transpose() << ",  y[" << idx
              << "] = " << y.row(idx) << std::endl;

    if (ExpectNear(xn, y.row(idx))) {
      x.block(idx, 0, 0, 2) << xn.transpose();
      x(idx, 3) = 1.F;
    } else {
      --idx;
      std::cout << "Data is not what is expected. Please try again"
                << std::endl;
    }
  }

  miss = (x.transpose() * x).ldlt().solve(x.transpose() * y);
  const utils::Mat3 misalignment = miss.block(0, 0, 3, 3).transpose();
  const utils::Vec3 bias = miss.block(3, 0, 1, 3).transpose();

  std::cout << "x = \n" << x << std::endl;
  std::cout << "The solution using normal equations is:\n" << miss << std::endl;
  std::cout << "misalignment:\n" << misalignment << std::endl;
  std::cout << "bias:\n" << bias << std::endl;

  SensorSpecs<3> calib_spec(spec);
  calib_spec.SetCalibration(misalignment, bias, utils::Vec3::Zero());
  return calib_spec;
}

SensorSpecs<3> CalibrateGyroscope(const ReadFunc& cb,
                                  const SensorSpecs<3>& spec) {
  std::cout << "Process of calibrating gyroscope" << std::endl;
  if (!GetUserApproval("Sit the sensor still")) {
    return spec;
  }
  const utils::Vec3 bias = GetAverage(cb);
  std::cout << "Gyro Bias: " << bias.transpose() << std::endl;

  SensorSpecs<3> calib_spec(spec);
  calib_spec.SetCalibration(utils::Mat3::Identity(), bias, utils::Vec3::Zero());
  return calib_spec;
}

SensorSpecs<3> CalibrateMagnetometer(const ReadFunc& cb,
                                     const SensorSpecs<3>& spec) {
  std::cout << "Process of calibrating accelerometer" << std::endl;
  if (!GetUserApproval("Wave device in a figure eight until done! To start")) {
    return spec;
  }
  Eigen::Matrix<utils::MATH_TYPE, kNumSamples, 3> data;
  data.setZero();
  // collect sampled data
  std::cout << "collecting data";
  constexpr auto packet_size = 10;
  for (Eigen::Index i = 0; i < data.rows(); i++) {
    data.row(i) << cb().transpose();
    navio::hardware_utils::Delay(ShortDelay);
    if (i % packet_size == 0) {
      std::cout << "." << std::flush;
    }
  }
  std::cout << std::endl;

  // find min max
  utils::Vec3 max;
  max << data.colwise().maxCoeff();
  utils::Vec3 min;
  min << data.colwise().minCoeff();

  // get average bias in counts
  const utils::Vec3 bias = (max + min) / 2.F;

  // get average max chord length in counts
  const utils::Vec3 max_chord = (max - min) / 2.F;
  const float avg_rad = max_chord.sum() / 3.F;
  const utils::Vec3 scale = avg_rad / max_chord.array();

  utils::Mat3 scale_mat = utils::Mat3::Zero();
  scale_mat.diagonal() = scale.array();
  SensorSpecs<3> calib_spec(spec);
  calib_spec.SetCalibration(scale_mat, bias, utils::Vec3::Zero());

  std::cout << "Mag scale:\n" << scale_mat << std::endl;
  std::cout << "Mag bias: " << bias.transpose() << std::endl;
  return calib_spec;
}
}  // namespace sensors::common::calibrate
