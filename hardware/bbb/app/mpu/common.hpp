#ifndef MPU_COMMON_HPP
#define MPU_COMMON_HPP
#include <cstdint>

/**
 * @brief Maximum I2C bus identifier. Default is 3 for a total of 3 busses.
 * This can be increased by the user for special cases.
 */
constexpr uint8_t I2C_MAX_BUS = 3;

// I2C bus and address definitions for beagle-bone blue
constexpr uint8_t MPU_BUS = 2;

#endif  // MPU_COMMON_HPP
