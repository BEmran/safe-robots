#ifndef HARDWARE_COMMON_COMM_I2C_HPP_
#define HARDWARE_COMMON_COMM_I2C_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "common/comm/communication_abs.hpp"

namespace hardware::common::comm {

class I2C : public CommunicationAbs {
 public:
  /**
   * @brief Construct a new I2C object
   *
   * @param debug enable/disable internal debug messages
   */
  explicit I2C(bool debug = false);

  /**
   * @brief Destroy the I2C object
   *
   */
  ~I2C();

  bool WriteByte(const uint8_t reg, const uint8_t data) const override;
  std::vector<uint8_t> ReadBytes(const uint8_t reg,
                                 const uint8_t count) const override;
  void Close() override;

  /**
   * @brief Initializes a bus and sets it to talk to a particular device
   * address.
   *
   * @param bus The bus
   * @param device_address The device address
   *
   * @return 0 on success or -1 on failure
   */
  bool Initialize(const uint8_t bus_num, const uint8_t device_address);

  /**
   * @brief Changes the device address's the bus is configured to talk to.
   *
   * @details Changing the device address in the I2C driver requires
   * a system call and is relatively slow. This function records which device
   * address the bus is set to and will only make the system call if the
   * requested address is different than the set address. This makes it safe to
   * call this function repeatedly with no performance penalty.
   *
   * @param bus The bus
   * @param device_address The new device address
   *
   * @return { description_of_the_return_value }
   */
  bool SetSlaveAddress(const uint8_t device_address);

  /**
   * @brief Fetches the current initialization status of I2C.
   *
   * @return true if initialized
   * @return false otherwise
   */
  inline bool IsInitialized() const {
    return initialized_;
  }

 protected:
  bool Open();

  void CloseImpl();
  /**
   * @brief Internal function to send data without prepending a register
   address.

   * @param data a vector of data to be sent
   * @return true if the data is sent successfully
   * @return false otherwise
   */
  bool SendData(const std::vector<uint8_t>& data) const;

  /**
   * @brief Internal function to read data from device
   *
   * @param count number of bytes to read
   * @return std::vector<uint8_t> a vector contains the response if
   * successful or empty otherwise
   */
  std::vector<uint8_t> GetResponse(const size_t count) const;

 private:
  bool initialized_{false};
  uint8_t bus_{0};
  uint8_t device_address_{0};
  int fd_{-1};
};
}  // namespace hardware::common::comm
#endif  // HARDWARE_COMMON_I2C_HPP_
