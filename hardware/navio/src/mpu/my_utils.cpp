#include "mpu/my_utils.hpp"

namespace mpu {

spi::SPI* GetSpi()
{
  static spi::SPI spi("/dev/spidev0.1", false);
  return &spi;
}

void Delay(uint32_t msec)
{
  usleep(msec * 1000);
}

void ResetSensors()
{
  // // store previous info ----------------------------------------
  // // master mode
  // const uint8_t master_mode = GetSpi()->ReadRegister(mpu9250::USER_CTRL);
  
  // // I2C configuration
  // const uint8_t master_ctrl = GetSpi()->ReadRegister(mpu9250::I2C_MST_CTRL); 

  // // wake up device
  // // GetSpi()->WriteRegister(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6),
  // // enable all sensors Clear sleep mode bit (6), enable all sensors
  // GetSpi()->WriteRegister(mpu9250::PWR_MGMT_1, 0x80);
  // Delay(100);  // Wait for all registers to reset

  // // get stable time source. Auto select clock source to be PLL gyroscope
  // // reference if ready else
  // GetSpi()->WriteRegister(mpu9250::PWR_MGMT_1, 0x01);
  // Delay(200);

  // // restore previous info ----------------------------------------
  // // master mode
  // GetSpi()->WriteRegister(mpu9250::USER_CTRL, master_mode);
  
  // // I2C configuration
  // GetSpi()->WriteRegister(mpu9250::I2C_MST_CTRL, master_ctrl);
}

int16_t To16Bit(const uint8_t msb, const uint8_t lsb)
{
  const auto m = static_cast<int>(msb);
  const auto l = static_cast<int>(lsb);
  return static_cast<int16_t>(m << 8 | l);
}

uint8_t ReadMPURegister(const uint8_t reg)
{
  uint8_t data[1] = {0};
  ReadMPURegisters(reg, 1, data);
  return data[0];
}

void ReadMPURegisters(const uint8_t reg, const uint8_t count, uint8_t * dest)
{
  GetSpi()->ReadRegisters(reg, count, dest);
}

uint8_t ReadAK8963Register(const uint8_t reg)
{
  uint8_t data[1] = {0};
  ReadAK8963Registers(reg, 1, data);
  return data[0];
}

void RequestReadAK8963Registers(const uint8_t reg, const uint8_t count)
{
  // set slave 0 to the AK8963 and set for read
  GetSpi()->WriteRegister(mpu9250::I2C_SLV0_ADDR,
                          ak8963::I2C_ADDR | mpu9250::I2C_READ_FLAG);
  // set the register to the desired AK8963 sub address
  GetSpi()->WriteRegister(mpu9250::I2C_SLV0_REG, reg);
  // enable I2C and request the bytes
  GetSpi()->WriteRegister(mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count);
  // takes some time for these registers to fill
  Delay(10);
}

void ReadAK8963Registers(const uint8_t reg, const uint8_t count, uint8_t * dest)
{
  RequestReadAK8963Registers(reg, count);
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
  GetSpi()->ReadRegisters(mpu9250::EXT_SENS_DATA_00, count, dest);
}

void WriteMPURegister(const uint8_t reg, const uint8_t data)
{
  GetSpi()->WriteRegister(reg, data);
}

void WriteAK8963Register(const uint8_t reg, const uint8_t data)
{
  constexpr uint8_t count = 1;
  // set slave 0 to the AK8963 and set for write
  GetSpi()->WriteRegister(mpu9250::I2C_SLV0_ADDR, ak8963::I2C_ADDR);
  // set the register to the desired AK8963 sub address
  GetSpi()->WriteRegister(mpu9250::I2C_SLV0_REG, reg);
  // store the data for write
  GetSpi()->WriteRegister(mpu9250::I2C_SLV0_DO, data);
  // enable I2C and send 1 byte
  GetSpi()->WriteRegister(mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count);
}

core::utils::Vec3 ArrayToVec3(const std::array<float, 3>& array)
{
  return core::utils::Vec3{array[0], array[1], array[2]};
}

std::array<float, 3> ApplyPost(const std::array<int16_t, 3>& full_bits, const Post post)
{
  std::array<float, 3> data;
  for (size_t i = 0; i < data.size(); i++)
  {
    data[i] = (static_cast<float>(full_bits[i]) * post.unit_conversion / post.sensitivity);
  }
  return  data;
}
} // namespace mpu 
