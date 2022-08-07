#ifndef CORE_UTILS_DATA_HPP
#define CORE_UTILS_DATA_HPP

// #include <Eigen/Core>
#include <iomanip>
#include <iostream>

namespace core::utils
{
// typedef Eigen::Vector3d Vec3;
// typedef Eigen::Quaterniond Quat;

/**
 * @brief general struct to hold 3 dimension vector
 *
 */
struct Vec3 {
  double x, y, z;  // value for each dimension

  /**
   * @brief Construct a new Vec3 object
   *
   * @param x_ value of x-axis
   * @param y_ value of y-axis
   * @param z_ value of z-axis
   */
  Vec3(const double x_ = 0.0, const double y_ = 0.0, const double z_ = 0.0)
      : x(x_), y(y_), z(z_) {}

  /**
   * @brief Construct a new Vec3 object from c-array
   *
   * @param arr c-array with size of three
   */
  explicit Vec3(const double* arr) {
    x = arr[0];
    y = arr[1];
    z = arr[2];
  }

  /**
   * @brief override operator[] to be used similar to c-array
   *
   * @param index index of the array
   * @return double& the corresponding index value
   */
  double& operator[](int index) {
    switch (index) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
      default:
        throw std::logic_error("In struct Vec3: index out of bound");
    }
  }
};

/**
 * @brief general struct to hold unit quaternion
 *
 */
struct Quat {
  double w, x, y, z;  // value for angle rotation and each unit vector

  /**
   * @brief Construct a new Quat object
   *
   * @param w_ value of angle of rotation
   * @param x_ value of unit vector i
   * @param y_ value of unit vector j
   * @param z_ value of unit vector k
   */
  Quat(const double w_ = 1.0, const double x_ = 0.0, const double y_ = 0.0,
       const double z_ = 0.0)
      : w(w_), x(x_), y(y_), z(z_) {}

  /**
   * @brief Construct a new Quat object from c-array
   *
   * @param arr c-array with size of four
   */
  explicit Quat(const double* arr) {
    w = arr[0];
    x = arr[1];
    y = arr[2];
    z = arr[3];
  }

  /**
   * @brief override operator[] to be used similar to c-array
   *
   * @param index index of the array
   * @return double& the corresponding index value
   */
  double& operator[](int index) {
    switch (index) {
      case 0:
        return w;
      case 1:
        return x;
      case 2:
        return y;
      case 3:
        return z;
      default:
        throw std::logic_error("In struct Quat: index out of bound");
    }
  }
};

std::ostream& operator<<(std::ostream& os, const Vec3& vec3);

std::ostream& operator<<(std::ostream& os, const Quat& quat);

class Data
{
  public:
  virtual ~Data() {};
  // std::string ToString();
  virtual void Print() = 0;
};

class AdcData : public Data
{
 public:
  void Print() override
  {
    std::cout << "Adc data: " << values
              << std::endl;
  }
  private:
  Vec3 values{0.0, 0.0, 0.0};
};

class BarData : public Data
{
 public:
  void Print() override
  {
    std::cout << "Barometer data: " << value
              << std::endl;
  }
 private:
  double value = 0.0;
};

 class GpsData : public Data
{
 public:
  void Print() override
  {
    std::cout << "GPS data: "
              << "\n- latitude: " << lat
              << "\n- longitude: " << lon
              << "\n- altitude: " << alt
              << std::endl;
  }

 private:
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
};

/**
 * @brief holds simple version of IMU sensor data
 *
 */
class ImuData : public Data
{
  public:
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
              << "\n- TaitBryan RPY(rad): " << TaitBryan
              << "\n- heading (rad): " << heading << "\n- Temp (C): " << temp
              << std::endl;
  }
  private:
  double temp = 0.0;     ///< thermometer, in units of degrees Celsius
  double heading = 0.0;  ///< fused heading filtered with gyro and accel data,
                         ///< same as Tait-Bryan yaw in radians
  Vec3 accel;            ///< accelerometer (XYZ) in units of m/s^2
  Vec3 gyro;             ///< gyroscope (XYZ) in units of degrees/s
  Vec3 mag;              ///< magnetometer (XYZ) in units of uT
  Quat quat;             ///< normalized quaternion
  Vec3 TaitBryan;        ///< Tait-Bryan angles (roll pitch yaw) in radians

};

// /**
//  * @brief holds simple version of mpu sensor data
//  *
//  */
// struct Led {
//   bool led1 = false;

//   /**
//    * @brief print led data details
//    *
//    */
//   void Print() {
//     std::cout << "LED data: "
//               << "\n- led1: " << BoolToChar(led1) << std::endl;
//   }
//   const char* BoolToChar(const bool b) { return b == true ? "On" : "Off"; }
// };

// /**
//  * @brief holds all types of data
//  *
//  */
// struct General {
//   struct Imu imu;
//   struct Led led;
// };
}
#endif  // CORE_UTILS_DATA_HPP