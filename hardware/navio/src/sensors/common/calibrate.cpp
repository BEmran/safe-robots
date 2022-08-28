#include "sensors/common/calibrate.hpp"

#include "navio/hardware_utils.hpp"

namespace sensors::common::calibrate
{
constexpr size_t kNumSamples = 1000;

utils::Vec3 GetAverage(const ReadFunc& cb)
{
  utils::Vec3 bias = utils::Vec3::Zero();
  std::cout << "collecting data";
  for (size_t i = 0; i < kNumSamples; i++)
  {
    const auto data = cb();
    bias += data;
    navio::hardware_utils::Delay(5);
    if (i % 10 == 0)
    {
      std::cout << "." << std::flush;
    }
  }
  std::cout << std::endl;
  return bias / kNumSamples;
}

SensorSpecs CalibrateAccelerometer(const ReadFunc& cb, const SensorSpecs& spec)
{
  Eigen::Matrix<utils::MATH_TYPE, 6, 3> y;
  y << +0.F, +0.F, +1.F,  //
      -1.F, +0.F, +0.F,   //
      +1.F, +0.F, +0.F,   //
      +0.F, -1.F, +0.F,   //
      +0.F, +1.F, +0.F,   //
      +0.F, +0.F, -1.F;
  Eigen::Matrix<utils::MATH_TYPE, 6, 4> x;
  x.setZero();
  Eigen::Matrix<utils::MATH_TYPE, 4, 3> miss;
  miss.setIdentity();
  std::cout << "Y = \n" << y << std::endl;
  std::cout << "x = \n" << x << std::endl;
  std::cout << "miss = \n" << miss << std::endl;
  const char* msg[] = {"face up",   "right side", "side left",
                       "nose down", "nose up",    "face down"};
  for (int i = 0; i < 6; i++)
  {
    std::cout << msg[i] << " and press enter.....";
    getchar();
    auto xn = GetAverage(cb) / spec.sensitivity;
    std::cout << "xn[" << i << "] = " << xn.transpose() << ",  y[" << i
              << "] = " << y.row(i) << std::endl;
    x.row(i) << xn[0], xn[1], xn[2], 1.F;
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

SensorSpecs CalibrateGyroscope(const ReadFunc& cb, const SensorSpecs& spec)
{
  const utils::Vec3 bias = GetAverage(cb);
  std::cout << "Gyro Bias: " << bias.transpose() << std::endl;

  SensorSpecs calib_spec(spec);
  calib_spec.SetCalibration(utils::Mat3::Identity(), bias, utils::Vec3::Zero());
  return calib_spec;
}

SensorSpecs CalibrateMagnetometer(const ReadFunc& cb, const SensorSpecs& spec)
{
  std::cout << "Mag Calibration: Wave device in a figure eight until done!"
            << std::endl;
  std::array<std::array<utils::MATH_TYPE, kNumSamples>, 3> data;

  // collect sampled data
  std::cout << "collecting data";
  for (size_t i = 0; i < kNumSamples; i++)
  {
    const auto mag = cb();
    data[0][i] = mag.x();
    data[1][i] = mag.y();
    data[2][i] = mag.z();
    navio::hardware_utils::Delay(5);
    if (i % 10 == 0)
    {
      std::cout << "." << std::flush;
    }
  }
  std::cout << std::endl;

  // find min max
  utils::Vec3 max = utils::Vec3::Zero();
  utils::Vec3 min = utils::Vec3::Zero();
  for (size_t i = 0; i < data.size(); i++)
  {
    auto ptr = std::minmax_element(data[i].begin(), data[i].end());
    min(i) = *(ptr.first);
    max(i) = *(ptr.second);
  }

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