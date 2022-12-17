// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "common/sensors/calibrate.hpp"

#include <string>
#include <string_view>

#include "common/sensors/utils.hpp"
#include "common/utils.hpp"
#include "core/utils/logger_macros.hpp"

namespace hardware::common::sensors {
namespace {
constexpr size_t kNumSamples = 1000;
constexpr uint32_t ShortDelay = 5;
constexpr auto kEpsilon = 0.025F;
constexpr int kQKey = static_cast<int>('q');
constexpr int kEscKey = 27;
constexpr std::string_view msg_request =
  " and press enter. To quit press q or Esc .....";
constexpr std::string_view kFaceMsg[] = {
  "Face Up", "Right Side", "Left Side", "Nose Down", "Nose Up", "Face Down"};

bool GetUserApproval(std::string_view msg) {
  SYS_LOG_INFO(msg) << msg_request;
  // Wait for user input
  const auto ch = getchar();
  if (ch == kQKey || ch == kEscKey) {
    SYS_LOG_INFO("User canceled the process");
    return false;
  }
  return true;
}

bool ExpectNear(const Vec3& v1, const Vec3& v2) {
  const Vec3 error = v1 - v2;
  const MATH_TYPE norm = error.norm();
  if (norm > kEpsilon) {
    SYS_LOG_WARN("The error between the two vectors are bigger that epsilon, ")
      << "error vector = " << error.transpose() << " norm = " << norm;
    return false;
  }
  return true;
}

Eigen::Matrix<MATH_TYPE, kNumSamples, 3> CollectData(const ReadFunc& cb) {
  Eigen::Matrix<MATH_TYPE, kNumSamples, 3> data;
  std::cout << "Collecting data, process will take "
            << kNumSamples * ShortDelay * 0.001 << " seconds";
  constexpr auto packet_size = 10;
  for (Eigen::Index i = 0; i < data.rows(); i++) {
    data.row(i) << cb().transpose();
    MilliDelay(ShortDelay);
    // print some info for users
    if (i % packet_size == 0) {
      std::cout << "." << std::flush;
    }
  }
  std::cout << std::endl;
  return data;
}

Vec3 GetAverage(const ReadFunc& cb) {
  const Eigen::Matrix<MATH_TYPE, kNumSamples, 3> data = CollectData(cb);
  Vec3 sum = Vec3::Zero();
  for (Eigen::Index i = 0; i < data.rows(); i++) {
    sum += data.row(i).transpose();
  }
  return sum / data.rows();
}

Eigen::Matrix<MATH_TYPE, 6, 3> AccelerometerExpectedData() {
  Eigen::Matrix<MATH_TYPE, 6, 3> y;  // NOLINT
  y.setZero();
  y(0, 2) = +1.F;  // NOLINT face up, z+
  y(1, 0) = -1.F;  // NOLINT right side, x-
  y(2, 0) = +1.F;  // NOLINT left side, x+
  y(3, 1) = -1.F;  // NOLINT nose down, y-
  y(4, 1) = +1.F;  // NOLINT nose up, y+
  y(5, 2) = -1.F;  // NOLINT face down, z-
  return y;
}

std::optional<Eigen::Matrix<MATH_TYPE, 6, 4>>
RobustlyCollectAverageDataForEachFace(const ReadFunc& cb,
                                      const SensorSpecs<3>& spec) {
  Eigen::Matrix<MATH_TYPE, 6, 4> x;  // NOLINT
  // data norm at each face expect to be 1
  x.block(0, 3, 6, 1) = Eigen::Matrix<MATH_TYPE, 6, 1>::Ones();
  const Eigen::Matrix<MATH_TYPE, 6, 3> y = AccelerometerExpectedData();

  Eigen::Index idx = 0;
  // loop through each face and collect average data
  while (idx < y.rows()) {
    if (!GetUserApproval(kFaceMsg[idx])) {
      return {};
    }

    // divide by sensitivity to collect raw value
    const Vec3 xn = GetAverage(cb) / spec.sensitivity;
    SYS_LOG_INFO() << "xn[" << idx << "] = " << xn.transpose() << ",  y[" << idx
                   << "] = " << y.row(idx);

    // If average value is accepted (near what expected) go to next face
    if (ExpectNear(xn, y.row(idx))) {
      x.block(idx, 0, 0, 2) << xn.transpose();
      idx++;
      SYS_LOG_INFO("Collected data is accepted. Continue for next face");
    } else {
      SYS_LOG_WARN("Collected Data has big error. Please try again");
    }
  }
  return x;
}

struct CalibrationResult {
  Mat3 misalignment = Mat3::Identity();
  Vec3 bias = Vec3::Zero();
};

SensorSpecs<3> UpdateOldCalibration(SensorSpecs<3> old,
                                    core::utils::InputMat misalignment,
                                    core::utils::InputMat bias,
                                    core::utils::InputMat offset) {
  // create new sensor calibration and copy result
  SensorSpecs<3> spec(old);
  spec.SetCalibration(misalignment, bias, offset);
  std::cout << "scale:\n" << misalignment << std::endl;
  std::cout << "bias: " << bias.transpose() << std::endl;
  std::cout << "offset: " << offset.transpose() << std::endl;
  return spec;
}

template <size_t SIZE>
CalibrationResult SolveLinearSystem(Eigen::Matrix<MATH_TYPE, SIZE, 3> y,
                                    Eigen::Matrix<MATH_TYPE, SIZE, 4> x) {
  Eigen::Matrix<MATH_TYPE, 4, 3> solution;  // NOLINT
  solution.setIdentity();
  solution = (x.transpose() * x).ldlt().solve(x.transpose() * y);
  CalibrationResult result;
  result.misalignment = solution.block(0, 0, 3, 3).transpose();
  result.bias = solution.block(3, 0, 1, 3).transpose();

  std::cout << "The solution using normal equations is:\n"
            << solution << std::endl;
  std::cout << "misalignment:\n" << result.misalignment << std::endl;
  std::cout << "bias:\n" << result.bias << std::endl;
  return result;
}

CalibrationResult
CalculateMagnetometer(Eigen::Matrix<MATH_TYPE, kNumSamples, 3> data) {
  // find min max
  Vec3 max;
  max << data.colwise().maxCoeff();

  Vec3 min;
  min << data.colwise().minCoeff();

  // get average bias in counts
  CalibrationResult result;
  result.bias = (max + min) / 2.F;

  // get average max chord length in counts
  const Vec3 max_chord = (max - min) / 2.F;
  const float avg_rad = max_chord.sum() / 3.F;
  const Vec3 scale = avg_rad / max_chord.array();

  result.misalignment.diagonal() = scale.array();
  return result;
}
}  // namespace

std::optional<SensorSpecs<3>>
CalibrateAccelerometer(const ReadFunc& cb, const SensorSpecs<3>& spec) {
  SYS_LOG_INFO("Process of calibrating accelerometer");

  auto average_data = RobustlyCollectAverageDataForEachFace(cb, spec);
  if (not average_data.has_value()) {
    return {};
  }

  const CalibrationResult solution =
    SolveLinearSystem<6>(AccelerometerExpectedData(), average_data.value());

  return UpdateOldCalibration(spec, solution.misalignment, solution.bias,
                              Vec3::Zero());
}

std::optional<SensorSpecs<3>> CalibrateGyroscope(const ReadFunc& cb,
                                                 const SensorSpecs<3>& spec) {
  SYS_LOG_INFO("Process of calibrating gyroscope");

  if (!GetUserApproval("Sit the sensor still")) {
    return {};
  }

  const Vec3 bias = GetAverage(cb);
  SYS_LOG_INFO("Gyro Bias: ") << bias.transpose();

  return UpdateOldCalibration(spec, Mat3::Identity(), bias, Vec3::Zero());
}

std::optional<SensorSpecs<3>>
CalibrateMagnetometer(const ReadFunc& cb, const SensorSpecs<3>& spec) {
  SYS_LOG_INFO("Process of calibrating Magnetometer");

  if (!GetUserApproval("Wave device in a figure eight until done! To "
                       "start")) {
    return {};
  }

  const Eigen::Matrix<MATH_TYPE, kNumSamples, 3> data = CollectData(cb);
  CalibrationResult result = CalculateMagnetometer(data);

  return UpdateOldCalibration(spec, result.misalignment, result.bias,
                              Vec3::Zero());
}
}  // namespace hardware::common::sensors
