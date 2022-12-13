// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_COMMON_COMM_COMMUNICATION_ABS_HPP_
#define HARDWARE_COMMON_COMM_COMMUNICATION_ABS_HPP_

#include <cstdint>
#include <vector>

namespace hardware::common::comm {

class CommunicationAbs {
 public:
  virtual ~CommunicationAbs() = default;
  explicit CommunicationAbs(bool debug = false) : debug_(debug) {
  }

  /**
   * @brief Writes a single byte to a specified register address.
   *
   * @param reg The register address
   * @param data  Single byte to be written
   * @return true if the data is written successfully
   * @return false otherwise
   */
  virtual bool WriteByte(const uint8_t reg, const uint8_t data) const = 0;

  /**
   * @brief Reads multiple bytes from a device register.
   *
   * @param reg The register address
   * @param count number of bytes to read
   * @return std::vector<uint8_t> a vector contains the response if
   * successful or empty otherwise
   */
  virtual std::vector<uint8_t> ReadBytes(const uint8_t reg,
                                         const uint8_t count) const = 0;

  /**
   * @brief Closes an I2C bus
   */
  virtual void Close() = 0;

 protected:
  /**
   * @brief indicate if debug is enabled
   *
   * @return true if enabled
   * @return false otherwise
   */
  inline bool IsDebug() const {
    return debug_;
  }

 private:
  bool debug_;
};
}  // namespace hardware::common::comm
#endif  // COMMON_COMMUNICATION_ABS_HPP_
