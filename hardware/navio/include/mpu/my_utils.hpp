#ifndef MPU_MY_UTILS_HPP
#define MPU_MY_UTILS_HPP

#include <stdint.h>
#include <unistd.h>
#include <assert.h>
#include <array>

#include "mpu/spi.hpp"
#include "mpu/mpu9250_register_map.h"
#include <core/utils/math.hpp>

namespace mpu
{

constexpr int max_bit_val = 32767;  // 2^16/2
constexpr int max_micro_tesla = 4912;

using core::utils::GRAVITY;
using core::utils::PI;

inline constexpr uint8_t operator"" _uc(unsigned long long arg) noexcept
{
  return static_cast<uint8_t>(arg);
}

struct Post
{
  float sensitivity;        // sensitivity
  float resolution;       // bit resolution = full_scale/ max_bit_count
  float unit_conversion;  // convert raw value to iso unit
  Post() : sensitivity(1), resolution(1.0), unit_conversion(1.0)
  {
  }
  Post(float sen, float res, float unit)
    : sensitivity(sen), resolution(res), unit_conversion(unit)
  {
  }
};

spi::SPI* GetSpi();

void Delay(uint32_t msec);

void ResetSensors();

// Turn the MSB and LSB into a signed 16-bit value
int16_t To16Bit(const uint8_t msb, const uint8_t lsb);

uint8_t ReadMPURegister(const uint8_t reg);

void ReadMPURegisters(const uint8_t reg, const uint8_t count, uint8_t* dest);

uint8_t ReadAK8963Register(const uint8_t reg);

void RequestReadAK8963Registers(const uint8_t reg, const uint8_t count);

void ReadAK8963Registers(const uint8_t reg, const uint8_t count, uint8_t* dest);

void WriteMPURegister(const uint8_t reg, const uint8_t data);

void WriteAK8963Register(const uint8_t reg, const uint8_t data);

// template <size_t N>
// std::array<int16_t, N> ReadRaw16Bits(const uint8_t reg)
// {
//   std::array<int16_t, N / 2> full_bits = {0};
//   // Read raw data registers sequentially into data array
//   uint8_t raw_data[N];
//   ReadMPURegisters(reg, N, &raw_data[0]);
//   // Turn the MSB and LSB into a signed 16-bit value
//   printf("full bits[size %u]: ", N);
//   for (size_t i = 0; i < N / 2; i += 2)
//   {
//     full_bits[i] = To16Bit(raw_data[i], raw_data[i + 1]);
//     printf("%d\t", full_bits[i]);
//   }
//   printf("\n");
//   return full_bits;
// }

// template <size_t N>
// std::array<float, N> Read16BitData(const uint8_t reg, const Post post)
// {
//   assert(N % 2 == 0 && "requested number of registers are not even");

//   auto full_bits = ReadRaw16Bits<N>(reg);
//   std::array<float, full_bits.size()> data = {0};

//   printf("data[size %u]: ", full_bits.size());
//   for (size_t i = 0; i < full_bits.size(); i++)
//   {
//     data[i] = static_cast<float>(full_bits[i]) * post.resolution *
//               post.unit_conversion;
//     printf("%f\t", data[i]);
//   }
//   printf("\n");
//   return data;
// }

// std::array<float, 3> ExtractSensorData(const uint8_t reg, const Post post)
// {
//   assert(N % 2 == 0 && "requested number of registers are not even");

//   auto full_bits = ReadRaw16Bits<N>(reg);
//   std::array<float, full_bits.size()> data = {0};

//   printf("data[size %u]: ", full_bits.size());
//   for (size_t i = 0; i < full_bits.size(); i++)
//   {
//     data[i] = static_cast<float>(full_bits[i]) * post.resolution *
//               post.unit_conversion;
//     printf("%f\t", data[i]);
//   }
//   printf("\n");
//   return data;
// }

// enum class BitsOrder
// {
//   BigIndian,
//   LittleIndian
// };

// std::vector<float> Extract16Bits(const float* begin_ptr,
//                                  const uint8_t count,
//                                  const BitsOrder order)
// {
//   std::vector<float> full_bits;
//   for (begin; begin != end; begin += 2)
//   {
//     const auto first = *begin;
//     const auto second = *(begin + 1);
//     switch (order)
//     {
//     case BitsOrder::BigIndian:
//       full_bits.push_back(To16Bit(first, second));
//       break;
//     case BitsOrder::LITTLE_ENDIAN:
//       full_bits.push_back(To16Bit(second, first));
//       break;
//     }
//   }
//   return full_bits;
// }

core::utils::Vec3 ArrayToVec3(const std::array<float, 3>& array);

std::array<float, 3> ApplyPost(const std::array<int16_t, 3>& full_bits, const Post post);

}  // namespace mpu

#endif  // MPU_MY_UTILS_HPP