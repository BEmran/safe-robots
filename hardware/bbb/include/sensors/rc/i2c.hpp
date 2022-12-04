// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef BBB_I2C_HPP_
#define BBB_I2C_HPP_

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
enum class ByteOrder {
  BIG_ENDIAN_ORDER,  // the most significant value is stored first at the lowest
                     // storage address.
  LITTLE_ENDIAN_ORDER  // the least significant value is stored first at the
                       // lowest storage address.
};

int16_t RegisterBytesToWord(const uint8_t reg0, const uint8_t reg1,
                            const ByteOrder order);

std::vector<int16_t> RegisterBytesToWords(const std::vector<uint8_t> bytes,
                                          const ByteOrder order);

std::pair<uint8_t, uint8_t> WordToRegisterBytes(const int16_t word,
                                                const ByteOrder order);

std::vector<uint8_t> WordsToRegisterBytes(const std::vector<int16_t> words,
                                          const ByteOrder order);
/**
 * @brief interface for the the Linux I2C driver.
 *
 */
class I2C {
 public:
  /**
   * @brief Construct a new I2C object
   *
   * @param bus i2c bus number
   */
  I2C(const size_t bus);

  /**
   * @brief Destroy the I2C object
   *
   */
  ~I2C();

  /**
   * @brief Initializes a bus and sets it to talk to a particular device
   * address.
   *
   * @param[in] dev_addr The device address
   *
   * @return true if successful
   * @return false otherwise
   */
  bool Initialize(const uint8_t dev_addr);

  /**
   * @brief Changes the device address the bus is configured to talk to.
   * @details This function records which device address the bus is set to and
   * will only make the system call if the requested address is different than
   * the set address.
   *
   * @param[in] dev_addr The new device address
   * @return true if successful
   * @return false otherwise
   */
  bool SetDeviceAddress(const uint8_t dev_addr);

  /**
   * @brief Close the bus and reset the I2CState
   *
   */
  void Close();

  /**
   * @brief Reads a single byte (8 bits) from a device register.
   *
   * @details This forward its argument to ReadBytes function with count of 1.
   *
   * @param[in] reg_addr The register address
   * @return a byte holds the response data
   */
  std::optional<uint8_t> ReadByte(const uint8_t reg_addr);

  /**
   * @brief Reads multiple bytes from a device register.
   *
   * @details This sends the device address and register address to be read
   * from before reading the response.
   *
   * @param[in] reg_addr The register address
   * @param[in] count number of bytes to read
   * @return a vector of bytes holds the response data
   */

  std::vector<uint8_t> ReadBytes(const uint8_t reg_addr, const size_t count);

  /**
   * @brief Reads a single word (16 bits) from a device register.
   *
   * @details This call forward its arguments to ReadWords function with count
   * of 1.
   *
   * @param[in] reg_addr The register address
   * @param[in] order ByteOrder to use when encoding the read word of data
   * @return a word holds the response data
   */
  std::optional<int16_t> ReadWord(const uint8_t reg_addr,
                                  const ByteOrder order);

  /**
   * @brief Reads multiple words (16 bytes each) from a device register.
   *
   * This sends the device address and register address to be read
   * from before reading the response, works for most i2c devices.
   *
   * @param[in] reg_addr The register address
   * @param[in] count Number of 16-bit words to read
   * @param[in] order ByteOrder to use when encoding the read word of data
   *
   * @return a vector of words holds the response data
   */
  std::vector<int16_t> ReadWords(const uint8_t reg_addr, const size_t count,
                                 const ByteOrder order);

  /**
   * @brief Writes a single byte to a specified register address.
   *
   * @details This call forward its arguments to WriteBytes function
   *
   * @param[in] reg_addr The register address
   * @param[in] data Single byte to be written
   *
   * @return true if data written successfully
   * @return false otherwise
   */
  bool WriteByte(const uint8_t reg_addr, const uint8_t data);

  /**
   * @brief Writes multiple bytes to a specified register address.
   *
   * @details This sends the device address and register address followed by
   * the actual data to be written.
   *
   * @param[in] reg_addr The register address to write to
   * @param data Vector of bytes of the data to be written
   *
   * @return true if data written successfully
   * @return false otherwise
   */
  bool WriteBytes(const uint8_t reg_addr, const std::vector<uint8_t>& data);

  /**
   * @brief Writes a single word (16 bits) to a specified register address.
   *
   * @details This call forward its arguments to WriteWords function
   *
   * @param[in] reg_addr The register address to write to
   * @param[in] data 16-bit word to be written
   * @param[in] order ByteOrder to use when decoding the word of data to be
   * written
   *
   * @return true if data written successfully
   * @return false otherwise
   */
  bool WriteWord(const uint8_t reg_addr, const int16_t data,
                 const ByteOrder order);

  /**
   * @brief Writes multiple words (16 bits each) to a specified register
   * address.
   *
   * @details This sends the device address and register address followed by
   * the actual data to be written.
   *
   * @param[in] reg_addr The register address
   * @param[in] data Vector of words of the data to be written
   * @param[in] order ByteOrder to use when decoding the word of data to be
   * written
   *
   * @return true if data written successfully
   * @return false otherwise
   */
  bool WriteWords(const uint8_t reg_addr, const std::vector<int16_t>& data,
                  const ByteOrder order);

  /**
   * @brief Sends exactly user-defined data without prepending a register
   * address.
   *
   * @details This forward its argument to SendBytes function
   *
   * @param[in] count Number of bytes to send
   * @param[in] data The data to be send
   *
   * @return true if data sent successfully
   * @return false otherwise
   */
  bool SendByte(const uint8_t data);

  /**
   * @brief Sends exactly user-defined data without prepending a register
   * address.
   *
   * @details Instead of automatically sending a device address before the data
   * which is typical for reading/writing registers, this function send only the
   * data given by the data argument. This is useful for more complicated IO
   * such as uploading firmware to a device.
   *
   * @param[in] data Vector of bytes of data to be send
   *
   * @return true if all data sent successfully
   * @return false otherwise
   */
  bool SendBytes(const std::vector<uint8_t>& data);

  // void SetStatus(I2CState state);
  // I2CState GetStatus();
  // std::optional<int> GetFd();

 protected:
  /**
   * @brief Configure the file descriptor
   *
   * @return true
   * @return false
   */
  bool Configure();
  bool Write(const std::vector<uint8_t>& data);

  /**
   * @brief Read the response
   *
   * @param count number of expected bytes to be red
   * @return std::vector<uint8_t> red bytes or empty if error
   */
  std::vector<uint8_t> Read(const size_t count);

 private:
  bool is_initialized_{false};
  int fd_{-1};
  /// @brief bus number
  size_t bus_{0};
  /// @brief The device address
  uint8_t dev_addr_{0};
  std::mutex i2c_mutex_;
};

/**
 * @brief manage all I2C
 *
 */
class I2CManager {
 public:
  static std::shared_ptr<I2CManager> GetInstance();
  std::shared_ptr<I2C> CreateI2C(const size_t bus, const uint8_t dev_addr);
  bool SetDeviceAddress(const size_t bus, const uint8_t dev_addr);
  bool Close(const size_t bus);

 protected:
  I2CManager();

 private:
  static std::shared_ptr<I2CManager> instance_;
  std::vector<std::shared_ptr<I2C>> i2cs_;
};
#endif  // RC_I2C_HPP
