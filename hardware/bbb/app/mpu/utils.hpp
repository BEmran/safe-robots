// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef BBB_UTILS_HPP_
#define BBB_UTILS_HPP_

#include <cstdint>  // for uint8_t types etc
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

/**
 * @brief size of i2c buffer in bytes for writing to registers.
 */
#define I2C_BUFFER_SIZE 128

/**
 * @brief describes the order in which a sequence of bytes is stored in computer
 * memory.
 */
enum class EndianByteOrder {
  BIG,    // the most significant value is stored first at the lowest
          // storage address.
  LITTLE  // the least significant value is stored first at the
          // lowest storage address.
};

int16_t RegisterBytesToWord(const uint8_t reg0, const uint8_t reg1,
                            const EndianByteOrder order);

std::vector<int16_t> RegisterBytesToWords(const std::vector<uint8_t> bytes,
                                          const EndianByteOrder order);

std::pair<uint8_t, uint8_t> WordToRegisterBytes(const int16_t word,
                                                const EndianByteOrder order);

std::vector<uint8_t> WordsToRegisterBytes(const std::vector<int16_t> words,
                                          const EndianByteOrder order);

#endif  // BBB_UTILS_HPP_
