#ifndef RC_I2C_H
#define RC_I2C_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "utils.hpp"

/**
 * @brief Maximum I2C bus identifier. Default is 5 for a total of 6 busses.
 * This can be increased by the user for special cases.
 */
#define I2C_MAX_BUS 5

/**
 * @brief size of i2c buffer in bytes for writing to registers. Only
 * increase if you know what you are doing.
 */
#define I2C_BUFFER_SIZE 128

// std::unique_ptr<I2C> CreateI2C(const int bus);

class I2C {
 public:
  I2C();
  ~I2C();

  /**
   * @brief Initializes a bus and sets it to talk to a particular device
   * address.
   *
   * @param[in] bus The bus
   * @param[in] device_address The device address
   *
   * @return 0 on success or -1 on failure
   */
  bool Initialize(const int bus_num, const uint8_t device_address);

  /**
   * @brief Changes the device address's the bus is configured to talk to.
   *
   * @details Changing the device address in the I2C driver requires
   * a system call and is relatively slow. This function records which device
   * address the bus is set to and will only make the system call if the
   * requested address is different than the set address. This makes it safe to
   * call this function repeatedly with no performance penalty.
   *
   * @param[in] bus The bus
   * @param[in] device_address The new device address
   *
   * @return { description_of_the_return_value }
   */
  bool SetSlaveAddress(const uint8_t device_address);

  /**
   * @brief Closes an I2C bus
   *
   * @return true if successfully closed
   * @return false otherwise
   */
  bool Close();

  /**
   * @brief Fetches the current lock's status
   *
   * @return true if locked
   * @return false otherwise
   */
  inline bool GetLock() const {
    return lock_;
  }

  /**
   * @brief Lock and unlock I2C, doesn't grantee no action on bus
   *
   * @param flag new lock status
   */
  inline void Lock(const bool flag) {
    lock_ = flag;
  }

  /**
   * @brief Fetches the current initialization status of I2C.
   *
   * @return true if initialized
   * @return false otherwise
   */
  inline bool Initialized() const {
    return initialized_;
  }

  /**
   * @brief Reads a single byte from a device register.
   *
   * @param[in] reg The register address
   * @return std::optional<uint8_t> an optional contains the response if
   * successful
   */
  std::optional<uint8_t> ReadByte(const uint8_t reg) const;

  /**
   * @brief Reads multiple bytes from a device register.
   *
   * @param[in] reg The register address
   * @param[in] count number of bytes to read
   * @return std::vector<uint8_t> a vector contains the response if
   * successful or empty otherwise
   */
  std::vector<uint8_t> ReadBytes(const uint8_t reg, const size_t count) const;

  /**
   * @brief Reads a single word (16 bits) from a device register.
   *
   * @param[in] reg The register address
   * @param[in] order byte order to use when converting the response bytes
   * @return std::optional<int16_t> an optional contains the response if
   * successful
   */
  std::optional<int16_t> ReadWord(const uint8_t reg,
                                  const EndianByteOrder order) const;

  /**
   * @brief Reads multiple words (16 bytes each) from a device register.
   *
   * @param[in] reg The register address
   * @param[in] count number of words to read
   * @param[in] order byte order to use when converting the response bytes
   * @return std::vector<int16_t> a vector contains the response if
   * successful or empty otherwise
   */
  std::vector<int16_t> ReadWords(const uint8_t reg, const size_t count,
                                 const EndianByteOrder order) const;

  /**
   * @brief Writes a single byte to a specified register address.
   *
   * @param[in] reg The register address
   * @param[in] data  Single byte to be written
   * @return true if the data is written successfully
   * @return false otherwise
   */
  bool WriteByte(const uint8_t reg, const uint8_t data) const;

  /**
   * @brief Writes multiple bytes to a specified register address.
   *
   * @param[in] reg The register address
   * @param[in] data a vector contains the data to be written
   * @return true if the data is written successfully
   * @return false otherwise
   */
  bool WriteBytes(const uint8_t reg, const std::vector<uint8_t>& data) const;

  /**
   * @brief Writes a single word (16 bits) to a specified register address.
   *
   * @param[in] reg The register address
   * @param[in] data a single 16-bit word to be written
   * @param[in] order byte order to use when decoding passed word
   * @return true if the data is written successfully
   * @return false otherwise
   */
  bool WriteWord(const uint8_t reg, const int16_t data,
                 const EndianByteOrder order) const;

  /**
   * @brief Writes multiple words (16 bits each) to a specified register
   * address.
   *
   * @param[in] reg The register address
   * @param[in] data a vector of 16-bit words to be written
   * @param[in] order byte order to use when decoding passed words
   * @return true if the data is written successfully
   * @return false otherwise
   */
  bool WriteWords(const uint8_t reg, const std::vector<int16_t>& data,
                  const EndianByteOrder order) const;

  /**
   * @brief Sends exactly user-defined data without prepending a register
   * address.
   *
   * @param[in] data a single byte to be sent
   * @return true if the data is sent successfully
   * @return false otherwise
   */
  bool SendByte(const uint8_t data) const;

  /**
   * @brief Sends exactly user-defined data without prepending a register
   * address.
   * @details Instead of automatically sending a device address before the
   * data which is typical for reading/writing registers, the rc_i2c_send_bytes
   * function send only the data given by the data argument.
   * @param[in] data a vector of data to be sent
   * @return true if the data is sent successfully
   * @return false otherwise
   */
  bool SendBytes(const std::vector<uint8_t>& data) const;

 protected:
  /**
   * @brief Fetches the current initialization status of I2C and print debug
   * message
   *
   * @return true if initialized
   * @return false otherwise
   */
  bool SanityCheck() const;

  /**
   * @brief Internal function to read data from device
   *
   * @param[in] count number of bytes to read
   * @return std::vector<uint8_t> a vector contains the response if
   * successful or empty otherwise
   */
  std::vector<uint8_t> Read(const size_t count) const;

 private:
  uint8_t device_address_{0};
  int fd_{0};
  uint8_t bus_{0};
  bool initialized_{false};
  mutable bool lock_{false};
};

/**
 * @brief Fetches the current lock state of the bus.
 *
 * @param[in] bus The bus ID
 *
 * @return Returns 0 if unlocked, 1 if locked, or -1 on error.
 */
bool rc_i2c_get_lock(const int bus);

#endif  // RC_I2C_H

///@} end group IO
