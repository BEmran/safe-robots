#include "sensors/rc/i2c.hpp"

#include <fcntl.h>          // open O_RDWR
#include <linux/i2c-dev.h>  // for IOCTL defs
#include <sys/ioctl.h>      // ioctl
#include <unistd.h>         // read, write, close

#include <core/utils/logger_macros.hpp>

/**
 * @brief Maximum I2C bus identifier for a total of 3 busses.
 */
constexpr uint8_t kI2CMaxBus = 2;

namespace {
/**
 * @brief simple struct hold the MSB and LSB of a word
 *
 */
struct TwoByte {
  /// @brief most significant bits (High 8-bits)
  uint8_t msb{0};

  /// @brief least significant bits (Low 8-bits)
  uint8_t lsb{0};

  TwoByte(const uint8_t msb_, const uint8_t lsb_) : msb{msb_}, lsb{lsb_} {
  }

  TwoByte(const int16_t word) {
    constexpr auto bit_shift = 8;
    constexpr auto mask = 0xFF;
    msb = static_cast<uint8_t>((word >> bit_shift) & mask);
    lsb = static_cast<uint8_t>(word & mask);
  }

  int16_t Word() const {
    constexpr auto bit_shift = 8;
    const int high = static_cast<int>(msb) << bit_shift;
    const int low = static_cast<int>(lsb);
    return static_cast<int16_t>(high | low);
  }
};

int16_t RegisterBytesToWordUsingLittleEndian(const uint8_t reg0,
                                             const uint8_t reg1) {
  const TwoByte two_byte(reg1, reg0);
  return two_byte.Word();
}

int16_t RegisterBytesToWordUsingBigEndian(const uint8_t reg0,
                                          const uint8_t reg1) {
  const TwoByte two_byte(reg0, reg1);
  return two_byte.Word();
}

std::pair<uint8_t, uint8_t>
WordToRegisterBytesUsingLittleEndian(const int16_t word) {
  const TwoByte two_byte{word};
  return {two_byte.lsb, two_byte.msb};
}

std::pair<uint8_t, uint8_t>
WordToRegisterBytesUsingBigEndian(const int16_t word) {
  const TwoByte two_byte{word};
  return {two_byte.msb, two_byte.lsb};
}

bool CheckBusRange(const size_t bus) {
  if (bus > kI2CMaxBus) {
    SYS_LOG_ERROR("i2c bus must be between 0 & " + std::to_string(kI2CMaxBus));
    return false;
  }
  return true;
}
}  // namespace

int16_t RegisterBytesToWord(const uint8_t reg0, const uint8_t reg1,
                            const ByteOrder order) {
  switch (order) {
    case ByteOrder::LITTLE_ENDIAN_ORDER:
      return RegisterBytesToWordUsingLittleEndian(reg0, reg1);
    case ByteOrder::BIG_ENDIAN_ORDER:
      return RegisterBytesToWordUsingBigEndian(reg0, reg1);
    default:
      SYS_LOG_WARN("ToWord, undefined ByteOrder");
      return 0;
  }
}

std::vector<int16_t> RegisterBytesToWords(const std::vector<uint8_t> bytes,
                                          const ByteOrder order) {
  if (bytes.size() % 2 == 0) {
    SYS_LOG_WARN("ToWords, the size of bytes vector should be even");
    return {};
  }

  std::vector<int16_t> words(bytes.size() / 2);
  for (size_t i = 0; i < words.size(); i++) {
    const size_t idx{i * 2};
    const uint8_t reg0{bytes[idx]};
    const uint8_t reg1{bytes[idx + 1]};
    words[i] = RegisterBytesToWord(reg0, reg1, order);
  }
  return words;
}

std::pair<uint8_t, uint8_t> WordToRegisterBytes(const int16_t word,
                                                const ByteOrder order) {
  switch (order) {
    case ByteOrder::LITTLE_ENDIAN_ORDER:
      return WordToRegisterBytesUsingLittleEndian(word);
    case ByteOrder::BIG_ENDIAN_ORDER:
      return WordToRegisterBytesUsingBigEndian(word);
    default:
      SYS_LOG_WARN("ToBytes, undefined ByteOrder");
      return {0, 0};
  }
}

std::vector<uint8_t> WordsToRegisterBytes(const std::vector<int16_t> words,
                                          const ByteOrder order) {
  std::vector<uint8_t> bytes(words.size() * 2);
  for (size_t i = 0; i < words.size(); i++) {
    const auto [reg0, reg1] = WordToRegisterBytes(words[i], order);
    const size_t idx{i * 2};
    bytes[idx] = reg0;
    bytes[idx + 1] = reg1;
  }
  return bytes;
}

I2C::I2C(const size_t bus) : bus_{bus} {
}

I2C::~I2C() {
  Close();
}

bool I2C::Initialize(const uint8_t dev_addr) {
  is_initialized_ = Configure() && SetDeviceAddress(dev_addr);
  return is_initialized_;
}

void I2C::Close() {
  // lock the bus during this operation
  const std::lock_guard<std::mutex> lock(i2c_mutex_);
  close(fd_);
  fd_ = -1;
  dev_addr_ = 0;
  is_initialized_ = false;
}

bool I2C::Configure() {
  if (fd_ != -1) {
    return true;
  }

  // lock the bus during this operation
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  // open file descriptor
  char str[16];
  sprintf(str, "/dev/i2c-%zu", bus_);
  fd_ = open(str, O_RDWR);
  if (fd_ == -1) {
    SYS_LOG_WARN("init, failed to open /dev/i2c");
    is_initialized_ = false;
    return false;
  }
  return true;
}

bool I2C::SetDeviceAddress(const uint8_t dev_addr) {
  if (fd_ == -1) {
    SYS_LOG_WARN("SetDeviceAddress, bus not configured yet");
    is_initialized_ = false;
    return false;
  }

  // if the device address is already correct, just return
  if (dev_addr_ == dev_addr) {
    return true;
  }

  // lock the bus during this operation
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  // if not, change it with ioctl
  if (ioctl(fd_, I2C_SLAVE, dev_addr_) < 0) {
    SYS_LOG_WARN("SetDeviceAddress, ioctl slave address change failed");
    is_initialized_ = false;
    return false;
  }

  dev_addr_ = dev_addr;
  return true;
}

std::optional<int16_t> I2C::ReadWord(const uint8_t reg_addr,
                                     const ByteOrder order) {
  const auto data = ReadWords(reg_addr, 1, order);
  if (data.empty()) {
    return {};
  }
  return data[0];
}

std::vector<int16_t> I2C::ReadWords(const uint8_t reg_addr, const size_t count,
                                    const ByteOrder order) {
  const size_t num_bytes_to_read = count * 2;
  const std::vector<uint8_t> buf = ReadBytes(reg_addr, num_bytes_to_read);
  if (buf.size() != num_bytes_to_read) {
    return {};
  }

  // form words from bytes and put into user's data array
  return RegisterBytesToWords(buf, order);
}

std::optional<uint8_t> I2C::ReadByte(const uint8_t reg_addr) {
  const std::vector<uint8_t> data = ReadBytes(reg_addr, 1);
  if (data.empty()) {
    return {};
  }
  return data[0];
}

std::vector<uint8_t> I2C::ReadBytes(const uint8_t reg_addr,
                                    const size_t count) {
  if (not SendByte(reg_addr)) {
    SYS_LOG_WARN("ReadBytes: failed to write to bus");
    return {};
  }

  // Read the response
  return Read(count);
}

bool I2C::WriteWord(const uint8_t reg_addr, const int16_t data,
                    const ByteOrder order) {
  return WriteWords(reg_addr, {data}, order);
}

bool I2C::WriteWords(const uint8_t reg_addr, const std::vector<int16_t>& data,
                     const ByteOrder order) {
  // assemble bytes to send
  std::vector<uint8_t> bytes_data = WordsToRegisterBytes(data, order);
  return WriteBytes(reg_addr, bytes_data);
}

bool I2C::WriteByte(const uint8_t reg_addr, const uint8_t data) {
  return WriteBytes(reg_addr, {data});
}

bool I2C::WriteBytes(const uint8_t reg_addr, const std::vector<uint8_t>& data) {
  // assemble array to send, starting with the register address
  std::vector<uint8_t> write_data(data.size() + 1);
  write_data[0] = reg_addr;
  std::copy(data.begin(), data.end(), write_data.begin() + 1);

  return SendBytes(write_data);
}

bool I2C::SendByte(const uint8_t data) {
  return SendBytes({data});
}

bool I2C::SendBytes(const std::vector<uint8_t>& data) {
  return Write(data);
}

bool I2C::Write(const std::vector<uint8_t>& data) {
  // sanity check
  if (not is_initialized_) {
    SYS_LOG_WARN("Write: bus not initialized yet");
    return false;
  }

  // lock the bus during this operation
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  // send the bytes
  const long ret = write(fd_, data.data(), data.size());

  // write should have returned the correct # bytes written
  if (ret != static_cast<long>(data.size())) {
    SYS_LOG_WARN("Write: failed to write to bus");
    return false;
  }
  return true;
}

std::vector<uint8_t> I2C::Read(const size_t count) {
  // sanity check
  if (not is_initialized_) {
    SYS_LOG_WARN("Write: bus not initialized yet");
    return {};
  }

  // lock the bus during this operation
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  // Read the response
  std::vector<uint8_t> data(count);
  const long ret = read(fd_, data.data(), count);
  if (ret != static_cast<long>(count)) {
    SYS_LOG_WARN("Read: received unexpected number of bytes from device");
    return {};
  }
  return data;
}

std::shared_ptr<I2CManager> I2CManager::instance_ = nullptr;

std::shared_ptr<I2CManager> I2CManager::GetInstance() {
  if (not instance_) {
    instance_ = std::make_shared<I2CManager>(I2CManager());
  }
  return instance_;
}

std::shared_ptr<I2C> I2CManager::CreateI2C(const size_t bus,
                                           const uint8_t dev_addr) {
  // sanity check
  if (bus > i2cs_.size()) {
    SYS_LOG_WARN("Close, wrong bus number");
    return nullptr;
  }

  // initialize
  if (not i2cs_[bus]->Initialize(dev_addr)) {
    SYS_LOG_WARN("failed to initialize i2c bus");
    return nullptr;
  }
  return i2cs_[bus];
}

bool I2CManager::SetDeviceAddress(const size_t bus, const uint8_t dev_addr) {
  // sanity check
  if (bus > i2cs_.size()) {
    SYS_LOG_WARN("SetDeviceAddress, wrong bus number");
    return false;
  }
  return i2cs_[bus]->SetDeviceAddress(dev_addr);
}

bool I2CManager::Close(const size_t bus) {
  // sanity check
  if (bus > i2cs_.size()) {
    SYS_LOG_WARN("SetDeviceAddress, wrong bus number");
    return false;
  }
  i2cs_[bus]->Close();
  return true;
}

I2CManager::I2CManager() {
  for (size_t bus_num = 0; bus_num < kI2CMaxBus + 1; bus_num++) {
    if (not CheckBusRange(bus_num)) {
      break;
    }
    i2cs_.push_back(std::make_shared<I2C>(bus_num));
  }
}