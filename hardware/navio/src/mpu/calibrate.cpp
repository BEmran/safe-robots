#include <mpu/calibrate.hpp>

namespace mpu
{
constexpr float accel_sen = max_bit_val / 16.F;

Vec3 GetAverage(const std::function<Vec3(void)>& cb)
{
  constexpr size_t samples = 1000;
  Vec3 bias = Vec3::Zero();
  std::cout << "collecting data";
  for (size_t i = 0; i < samples; i++)
  {
    const auto data = cb();
    bias += data;
    Delay(5);
    if (i % 10 == 0)
    {
      std::cout << "." << std::flush;
    }
  }
  std::cout << std::endl;
  return bias / samples;
}

SensorSpecs CalibrateAccelerometer(const std::function<Vec3(void)>& cb,
                                   const SensorSpecs& spec)
{
  Eigen::Matrix<MATH_TYPE, 6, 3> y;
  y << +0.F, +0.F, +1.F,  //
      -1.F, +0.F, +0.F,   //
      +1.F, +0.F, +0.F,   //
      +0.F, -1.F, +0.F,   //
      +0.F, +1.F, +0.F,   //
      +0.F, +0.F, -1.F;
  Eigen::Matrix<MATH_TYPE, 6, 4> x;
  x.setZero();
  Eigen::Matrix<MATH_TYPE, 4, 3> miss;
  miss.setIdentity();
  std::cout << "Y = \n" << y << std::endl;
  std::cout << "x = \n" << x << std::endl;
  std::cout << "miss = \n" << miss << std::endl;
  const char* msg[] = {"face up",   "right side", "side left",
                       "nose down", "nose up",    "face down"};
  for (size_t i = 0; i < 6; i++)
  {
    std::cout << msg[i] << " and press enter.....";
    getchar();
    auto xn = GetAverage(cb) / accel_sen;
    std::cout << "xn[" << i << "] = " << xn.transpose() << ",  y[" << i
              << "] = " << y.row(i) << std::endl;
    x.row(i) << xn[0], xn[1], xn[2], 1.F;
  }

  miss = (x.transpose() * x).ldlt().solve(x.transpose() * y);
  std::cout << "x = \n" << x << std::endl;
  std::cout << "The solution using normal equations is:\n" << miss << std::endl;

  const Mat3 misalignment = miss.block(0, 0, 3, 3).transpose();
  const Vec3 bias = miss.block(3, 0, 1, 3).transpose();
  std::cout << "misalignment:\n" << misalignment << std::endl;
  std::cout << "bias:\n" << bias << std::endl;

  SensorSpecs calib_spec(spec);
  calib_spec.SetCalibration(misalignment, bias, Vec3::Zero());
  return calib_spec;
}

SensorSpecs CalibrateGyroscope(const std::function<Vec3(void)>& cb,
                               const SensorSpecs& spec)
{
  const Vec3 bias = GetAverage(cb);
  std::cout << "Gyro Bias: " << bias.transpose() << std::endl;

  SensorSpecs calib_spec(spec);
  calib_spec.SetCalibration(Mat3::Identity(), bias, Vec3::Zero());
  return calib_spec;
}

SensorSpecs CalibrateMagnetometer(const std::function<Vec3(void)>& cb,
                                  const SensorSpecs& spec)
{
  constexpr size_t samples = 1000;
  std::cout << "Mag Calibration: Wave device in a figure eight until done!"
            << std::endl;
  std::array<std::array<MATH_TYPE, samples>, 3> data;
  
  // collect sampled data
  std::cout << "collecting data";
  for (size_t i = 0; i < samples; i++)
  {
    const auto mag = cb();
    data[0][i] = mag.x();
    data[1][i] = mag.y();
    data[2][i] = mag.z();
    Delay(5);
    if (i % 10 == 0)
    {
      std::cout << "." << std::flush;
    }
  }
  std::cout << std::endl;

  // find min max
  Vec3 max = Vec3::Zero();
  Vec3 min = Vec3::Zero();
  for (size_t i = 0; i < data.size(); i++)
  {
    auto ptr = std::minmax_element(data[i].begin(), data[i].end());
    min(i) = *(ptr.first);
    max(i) = *(ptr.second);
  }

  // get average bias in counts
  const Vec3 bias = (max + min) / 2.F;

  // get average max chord length in counts
  const Vec3 max_chord = (max - min) / 2.F;
  const float avg_rad = max_chord.sum() / 3.F;
  const Vec3 scale = avg_rad / max_chord.array();

  Mat3 scale_mat = Mat3::Zero();
  scale_mat.diagonal() = scale.array();
  SensorSpecs calib_spec(spec);
  calib_spec.SetCalibration(scale_mat, bias, Vec3::Zero());

  std::cout << "Mag scale:\n" << scale_mat << std::endl;
  std::cout << "Mag bias: " << bias.transpose() << std::endl;
  return calib_spec;
}
}  // namespace mpu