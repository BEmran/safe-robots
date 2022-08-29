// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "sensors/common/calibrate.hpp"

#include "navio/hardware_utils.hpp"

namespace sensors::common::calibrate {
constexpr size_t kNumSamples = 1000;
constexpr auto ShortDelay = 5;
constexpr auto kEscKey = 'q';
constexpr auto kEpsilon = 0.025F;

constexpr const char* kFaceMsg[] = {"face up",   "right side", "left side",
                                    "nose down", "nose up",    "face down"};
bool GetUserApproval(const char* msg) {
  std::cout << msg << " and press enter or q to quit .....";
  const auto ch = getchar();
  return ch != kEscKey;
}

bool CheckCollectedDataIsNearExpectation(const utils::Vec3& v1,
                                         const utils::Vec3& v2) {
  utils::Vec3 error = v1 - v2;
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

SensorSpecs CalibrateAccelerometer(const ReadFunc& cb,
                                   const SensorSpecs& spec) {
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
      std::cout << "user canceled the process" << std::endl;
      return spec;
    }
    auto xn = GetAverage(cb) / spec.sensitivity;

    std::cout << "xn[" << idx << "] = " << xn.transpose() << ",  y[" << idx
              << "] = " << y.row(idx) << std::endl;

    if (!CheckCollectedDataIsNearExpectation(xn, y.row(idx))) {
      std::cout << "Data is not what is expected. Please try again"
                << std::endl;
      --idx;
    } else {
      x.row(idx) << xn.transpose(), 1.F;
    }
  }

  miss = (x.transpose() * x).ldlt().solve(x.transpose() * y);
  std::cout << "x = \n" << x << std::endl;
  std::cout << "The solution using normal equations is:\n" << miss << std::endl;

  const utils::Mat3 misalignment = miss.block(0, 0, 3, 3).transpose();
  const utils::Vec3 bias = miss.block(3, 0, 1, 3).transpose();
  std::cout << "misalignment:\n" << misalignment << std::endl;
  std::cout << "bias:\n" << bias << std::endl;

  SensorSpecs calib_spec(spec);
  calib_spec.SetCalibration(misalignment, bias, utils::Vec3::Zero());
  return calib_spec;
}

SensorSpecs CalibrateGyroscope(const ReadFunc& cb, const SensorSpecs& spec) {
  const utils::Vec3 bias = GetAverage(cb);
  std::cout << "Gyro Bias: " << bias.transpose() / spec.sensitivity
            << std::endl;

  SensorSpecs calib_spec(spec);
  calib_spec.SetCalibration(utils::Mat3::Identity(), bias, utils::Vec3::Zero());
  return calib_spec;
}

SensorSpecs CalibrateMagnetometer(const ReadFunc& cb, const SensorSpecs& spec) {
  std::cout << "Mag Calibration: Wave device in a figure eight until done!"
            << std::endl;
  // std::array<std::array<utils::MATH_TYPE, kNumSamples>, 3> data;
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
  min << data.colwise().maxCoeff();

  // get average bias in counts
  const utils::Vec3 bias = (max + min) / 2.F;

  // get average max chord length in counts
  const utils::Vec3 max_chord = (max - min) / 2.F;
  const float avg_rad = max_chord.sum() / 3.F;
  const utils::Vec3 scale = avg_rad / max_chord.array();

  utils::Mat3 scale_mat = utils::Mat3::Zero();
  scale_mat.diagonal() = scale.array();
  SensorSpecs calib_spec(spec);
  calib_spec.SetCalibration(scale_mat, bias, utils::Vec3::Zero());

  std::cout << "Mag scale:\n" << scale_mat << std::endl;
  std::cout << "Mag bias: " << bias.transpose() << std::endl;
  return calib_spec;
}
}  // namespace sensors::common::calibrate
