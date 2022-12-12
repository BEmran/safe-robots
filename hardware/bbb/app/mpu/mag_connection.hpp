#ifndef RC_MAG_CONNECTION_HPP
#define RC_MAG_CONNECTION_HPP

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "i2c.hpp"
#include "mag_def.hpp"

class MagConnection {
 public:
  virtual bool Initialize(std::shared_ptr<I2C> i2c, const MagConfig& conf) = 0;
  virtual bool WriteByte(const uint8_t reg, const uint8_t data) const = 0;
  virtual std::vector<uint8_t> ReadBytes(const uint8_t reg,
                                         const uint8_t count) const = 0;
  inline std::optional<uint8_t> ReadByte(const uint8_t reg) const {
    const auto result = ReadBytes(reg, 1);
    if (result.empty()) {
      return {};
    }
    return result[0];
  };

 protected:
  std::shared_ptr<I2C> i2c_{nullptr};
};

#endif  // RC_MAG_HPP
