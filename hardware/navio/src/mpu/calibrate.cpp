#include <mpu/calibrate.hpp>

namespace mpu
{
constexpr float accel_sen = max_bit_val / 16.F;

Vec3 GetAverage(const std::function<Vec3(void)>& cb)
{
  constexpr size_t samples = 1000;
  Vec3 bias = Vec3::Zero();
  for (int i = 0; i < samples; i++)
  {
    const auto data = cb();
    // std::cout << "\t["<< i << "] :" << data.transpose();
    bias += data;
    Delay(2);
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
  Eigen::Matrix<MATH_TYPE, 4, 3> M;
  M.setIdentity();
  std::cout << "Y = \n" << y << std::endl;
  std::cout << "x = \n" << x << std::endl;
  std::cout << "M = \n" << M << std::endl;
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

  M = (x.transpose() * x).ldlt().solve(x.transpose() * y);
  std::cout << "x = \n" << x << std::endl;
  std::cout << "The solution using normal equations is:\n" << M << std::endl;

  const Mat3 misalignment = M.block(0, 0, 3, 3).transpose();
  const Vec3 bias = M.block(3, 0, 1, 3).transpose();
  std::cout << "misalignment:\n" << misalignment << std::endl;
  std::cout << "bias:\n" << bias << std::endl;

  SensorSpecs calib_spec(spec);
  calib_spec.SetMisalignment(misalignment);
  calib_spec.SetBias(bias);
  return calib_spec;
}

SensorSpecs CalibrateGyroscope(const std::function<Vec3(void)>& cb,
                               const SensorSpecs& spec)
{
  const Vec3 bias = GetAverage(cb);
  std::cout << "Gyro Bias: " << bias.transpose() << std::endl;
  
  SensorSpecs calib_spec(spec);
  calib_spec.SetBias(bias);
  return calib_spec;
}

// void CalibrateMagnetometer(std::shared_ptr<ImuSensorModule> sensor)
// {}

// void calibrate(void)
// {
//   // // reset device
//   // writeMPURegister(PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit;
//   toggle
//   //                                      // reset device
//   // delay(100);

//   // // get stable time source; Auto select clock source to be PLL gyroscope
//   // // reference if ready else use the internal oscillator, bits 2:0 = 001
//   // writeMPURegister(PWR_MGMT_1, INV_CLK_PLL);
//   // writeMPURegister(PWR_MGMT_2, 0x00);
//   // delay(200);

//   // // Configure device for bias calculation
//   // writeMPURegister(INT_ENABLE, 0x00);    // Disable all interrupts
//   // writeMPURegister(FIFO_EN, 0x00);       // Disable FIFO
//   // writeMPURegister(PWR_MGMT_1, 0x00);    // Turn on internal clock source
//   // writeMPURegister(I2C_MST_CTRL, 0x00);  // Disable I2C master
//   // writeMPURegister(USER_CTRL, 0x00);     // Disable FIFO and I2C master
//   modes
//   // writeMPURegister(USER_CTRL, 0x0C);     // Reset FIFO and DMP
//   // delay(15);

//   // // Configure gyro and accelerometer for bias calculation
//   // writeMPURegister(CONFIG, 0x01);  // Set low-pass filter to 188 Hz
//   // writeMPURegister(SMPLRT_DIV, _sampleRateDivisor);  // Set sample rate to
//   1 kHz
//   // writeMPURegister(GYRO_CONFIG, _gScale << 3);
//   // writeMPURegister(ACCEL_CONFIG, _aScale << 3);

// //   uint16_t gyrosensitivity = 131;     // = 131 LSB/degrees/sec
// //   uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

//   // Configure FIFO to capture accelerometer and gyro data for bias
//   calculation writeMPURegister(USER_CTRL, 0x40);  // Enable FIFO
//   writeMPURegister(FIFO_EN, 0x78);  // Enable gyro and accelerometer sensors
//   for
//                                     // FIFO  (max size 512 bytes in MPU-9150)
//   delay(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

//   // At end of sample accumulation, turn off FIFO sensor read
//   uint8_t data[12];  // data array to hold accelerometer and gyro x, y, z,
//   data writeMPURegister(FIFO_EN,
//                    0x00);  // Disable gyro and accelerometer sensors for FIFO
//   readMPURegisters(FIFO_COUNTH, 2, &data[0]);  // read FIFO sample count
//   uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
//   uint16_t packet_count = fifo_count / 12;  // How many sets of full gyro and
//                                             // accelerometer data for
//                                             averaging

//   int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
//   for (int k = 0; k < packet_count; k++)
//   {
//     int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
//     readMPURegisters(FIFO_R_W, 12, &data[0]);  // read data for averaging
//     accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) |
//                               data[1]);  // Form signed 16-bit integer for
//                               each
//                                          // sample in FIFO
//     accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
//     accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
//     gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
//     gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
//     gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

//     accel_bias[0] +=
//         (int32_t)accel_temp[0];  // Sum individual signed 16-bit biases to
//         get
//                                  // accumulated signed 32-bit biases
//     accel_bias[1] += (int32_t)accel_temp[1];
//     accel_bias[2] += (int32_t)accel_temp[2];
//     gyro_bias[0] += (int32_t)gyro_temp[0];
//     gyro_bias[1] += (int32_t)gyro_temp[1];
//     gyro_bias[2] += (int32_t)gyro_temp[2];
//   }
//   accel_bias[0] /=
//       (int32_t)packet_count;  // Normalize sums to get average count biases
//   accel_bias[1] /= (int32_t)packet_count;
//   accel_bias[2] /= (int32_t)packet_count;
//   gyro_bias[0] /= (int32_t)packet_count;
//   gyro_bias[1] /= (int32_t)packet_count;
//   gyro_bias[2] /= (int32_t)packet_count;

//   if (accel_bias[2] > 0L)
//   {
//     // Remove gravity from the z-axis accelerometer bias calculation
//     accel_bias[2] -= _accelsensitivity;
//   }
//   else
//   {
//     accel_bias[2] += _accelsensitivity;
//   }

//   // Construct the gyro biases for push to the hardware gyro bias registers,
//   // which are reset to zero upon device startup
//   data[0] = (-gyro_bias[0] / 4 >> 8) &
//             0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to
//                    // expected bias input format
//   data[1] = (-gyro_bias[0] / 4) & 0xFF;  // Biases are additive, so change
//   sign
//                                          // on calculated average gyro biases
//   data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
//   data[3] = (-gyro_bias[1] / 4) & 0xFF;
//   data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
//   data[5] = (-gyro_bias[2] / 4) & 0xFF;

//   // Push gyro biases to hardware registers
//   pushGyroBiases(data);

//   // Output scaled gyro biases for display in the main program
//   _gyroBias[0] = (float)gyro_bias[0] / (float)_gyrosensitivity;
//   _gyroBias[1] = (float)gyro_bias[1] / (float)_gyrosensitivity;
//   _gyroBias[2] = (float)gyro_bias[2] / (float)_gyrosensitivity;

//   // Construct the accelerometer biases for push to the hardware
//   accelerometer
//   // bias registers. These registers contain factory trim values which must
//   be
//   // added to the calculated accelerometer biases; on boot up these registers
//   // will hold non-zero values. In addition, bit 0 of the lower byte must be
//   // preserved since it is used for temperature compensation calculations.
//   // Accelerometer bias registers expect bias input as 2048 LSB per g, so
//   that
//   // the accelerometer biases calculated above must be divided by 8.

//   int32_t accel_bias_reg[3] = {0, 0, 0};  // A place to hold the factory
//                                           // accelerometer trim biases

//   readAccelOffsets(data, accel_bias_reg);

//   uint32_t mask = 1uL;  // Define mask for temperature compensation bit 0 of
//                         // lower byte of accelerometer bias registers
//   uint8_t mask_bit[3] = {0, 0, 0};  // Define array to hold mask bit for each
//                                     // accelerometer bias axis

//   for (int k = 0; k < 3; k++)
//   {
//     if ((accel_bias_reg[k] & mask))
//       mask_bit[k] = 0x01;  // If temperature compensation bit is set, record
//                            // that fact in mask_bit
//   }

//   // Construct total accelerometer bias, including calculated average
//   // accelerometer bias from above
//   accel_bias_reg[0] -=
//       (accel_bias[0] / 8);  // Subtract calculated averaged accelerometer
//       bias
//                             // scaled to 2048 LSB/g (16 g full scale)
//   accel_bias_reg[1] -= (accel_bias[1] / 8);
//   accel_bias_reg[2] -= (accel_bias[2] / 8);

//   data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
//   data[1] = (accel_bias_reg[0]) & 0xFF;
//   data[1] =
//       data[1] | mask_bit[0];  // preserve temperature compensation bit when
//                               // writing back to accelerometer bias registers
//   data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
//   data[3] = (accel_bias_reg[1]) & 0xFF;
//   data[3] =
//       data[3] | mask_bit[1];  // preserve temperature compensation bit when
//                               // writing back to accelerometer bias registers
//   data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
//   data[5] = (accel_bias_reg[2]) & 0xFF;
//   data[5] =
//       data[5] | mask_bit[2];  // preserve temperature compensation bit when
//                               // writing back to accelerometer bias registers

//   // Apparently this is not working for the acceleration biases in the
//   MPU-9250
//   // Are we handling the temperature correction bit properly?
//   // Push accelerometer biases to hardware registers
//   //  writeMPURegister(XAOffsetH(), data[0]);
//   //  writeMPURegister(XA_OFFSET_L, data[1]);
//   //  writeMPURegister(YAOffsetH(), data[2]);
//   //  writeMPURegister(YA_OFFSET_L, data[3]);
//   //  writeMPURegister(ZAOffsetH(), data[4]);
//   //  writeMPURegister(ZA_OFFSET_L, data[5]);

//   // Output scaled accelerometer biases for display in the main program
//   _accelBias[0] = (float)accel_bias[0] / (float)_accelsensitivity;
//   _accelBias[1] = (float)accel_bias[1] / (float)_accelsensitivity;
//   _accelBias[2] = (float)accel_bias[2] / (float)_accelsensitivity;
// }
}  // namespace mpu