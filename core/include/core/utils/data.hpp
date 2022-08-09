#ifndef CORE_UTILS_DATA_HPP
#define CORE_UTILS_DATA_HPP

// #include <Eigen/Core>
#include <iomanip>
#include <iostream>
#include <array>
#include <algorithm>
#include <assert.h>

namespace core::utils
{
// typedef Eigen::Vector3d Vec3;
// typedef Eigen::Quaterniond Quat;

/**
 * @brief general struct to hold 3 dimension vector
 *
 */
struct Vec3
{
  std::array<double, 3> data;
  /**
   * @brief Construct a new Vec3 object and initialize it with zeros
   *
   */
  Vec3() : data({0.0, 0.0, 0.0})
  {
  }

  /**
   * @brief Construct a new Vec3 object
   *
   * @param x value of x-axis
   * @param y value of y-axis
   * @param z value of z-axis
   */
  Vec3(const double x, const double y, const double z) : data({x, y, z})
  {
  }

  /**
   * @brief Construct a new Vec3 object from c-array
   *
   * @param arr c-array with size of three
   */
  template <class T, std::size_t N>
  explicit Vec3(const T (&arr)[N])
  {
    assert(N==3);
    std::copy(std::begin(arr), std::end(arr), data.begin());
  }

  /**
   * @brief override operator[] to be used similar to c-array
   *
   * @param index index of the array
   * @return double& the corresponding index value
   */
  inline double& operator[](const size_t index)
  {
    assert(index < 3);
    return data[index];
  }

  /**
   * @brief override operator[] to be used similadr to c-array
   *
   * @param index index of the array
   * @return double the corresponding index value
   */
  inline double operator[](const size_t index) const
  {
    assert(index < 3);
    return data[index];
  }

  inline double& x()
  {
    return data[0];
  }
  inline double& y()
  {
    return data[1];
  }
  inline double& z()
  {
    return data[2];
  }
  inline double x() const
  {
    return data[0];
  }
  inline double y() const
  {
    return data[1];
  }
  inline double z() const
  {
    return data[2];
  }
};

/**
 * @brief general struct to hold unit quaternion
 *
 */
struct Quat
{
  double angle = 1;  // value for angle rotation
  Vec3 vector = {0, 0, 0};        // value for unit vector
  /**
   * @brief Construct a new Quat object
   *
   * @param w value of angle of rotation
   * @param x value of unit vector i
   * @param y value of unit vector j
   * @param z value of unit vector k
   */
  Quat(const double w, const double x, const double y, const double z)
    : angle(w), vector({x, y, z})
  {
  }
  /**
   * @brief Construct a new Quat object
   *
   * @param rot angle of rotation
   * @param unit_vec unit vector
   */
  Quat(const double rot = 1, const Vec3& unit_vec = Vec3())
    : angle(rot), vector(unit_vec)
  {
  }
};

std::ostream& operator<<(std::ostream& os, const Vec3& vec3);

std::ostream& operator<<(std::ostream& os, const Quat& quat);

struct Data
{
 public:
  virtual ~Data(){};
  virtual void Clear() = 0;
  // std::string ToString();
  virtual void Print() = 0;
};

struct AdcData : public Data
{
  Vec3 values{0.0, 0.0, 0.0};

  void Print() override
  {
    std::cout << "Adc data: " << values << std::endl;
  }

  void Clear() override
  {
    values = Vec3();
  }
};

struct BarData : public Data
{
  double value = 0.0;
  void Print() override
  {
    std::cout << "Barometer data: " << value << std::endl;
  }
  void Clear() override
  {
    value = 0.0;
  }
};

struct GpsData : public Data
{
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;

  void Print() override
  {
    std::cout << "GPS data: "
              << "\n- latitude: " << lat << "\n- longitude: " << lon
              << "\n- altitude: " << alt << std::endl;
  }

  void Clear() override
  {
    lat = 0.0;
    lon = 0.0;
    alt = 0.0;
  }
};

/**
 * @brief holds simple version of IMU sensor data
 *
 */
struct ImuData : public Data
{
  double temp = 0.0;     ///< thermometer, in units of degrees Celsius
  double heading = 0.0;  ///< fused heading filtered with gyro and accel data,
                         ///< same as Tait-Bryan yaw in radians
  Vec3 accel;            ///< accelerometer (XYZ) in units of m/s^2
  Vec3 gyro;             ///< gyroscope (XYZ) in units of degrees/s
  Vec3 mag;              ///< magnetometer (XYZ) in units of uT
  Quat quat;             ///< normalized quaternion
  Vec3 tait_bryan;       ///< Tait-Bryan angles (roll pitch yaw) in radians

  /**
   * @brief print imu data details
   *
   */
  void Print() override
  {
    std::cout << "IMU data: "
              << "\n- Accel XYZ(m/s^2): " << accel
              << "\n- Gyro  XYZ(rad/s): " << gyro
              << "\n- Mag Field XYZ(uT): " << mag << "\n- quat  WXYZ: " << quat
              << "\n- TaitBryan RPY(rad): " << tait_bryan
              << "\n- heading (rad): " << heading << "\n- Temp (C): " << temp
              << std::endl;
  }

  void Clear() override
  {
    temp = 0.0;
    heading = 0.0;
    accel = Vec3();
    gyro = Vec3();
    mag = Vec3();
    quat = Quat();
    tait_bryan = Vec3();
  }
};

}  // namespace core::utils
#endif  // CORE_UTILS_DATA_HPP