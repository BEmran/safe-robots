#ifndef RC_MAG_DIRECT_HPP
#define RC_MAG_DIRECT_HPP

#include <cstdint>
#include <vector>

#include "i2c.hpp"
#include "mag_connection.hpp"

class MagDirect : public MagConnection {
 public:
  bool Initialize(std::shared_ptr<I2C> i2c, const MagConfig& conf) override;
  bool WriteByte(const uint8_t reg, const uint8_t data) const override;
  std::vector<uint8_t> ReadBytes(const uint8_t reg,
                                 const uint8_t count) const override;
};

#endif  // RC_MAG_DIRECT_HPP
