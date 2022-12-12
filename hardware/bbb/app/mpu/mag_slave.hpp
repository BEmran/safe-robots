#ifndef RC_MAG_SLAVE_HPP
#define RC_MAG_SLAVE_HPP

#include <cstdint>
#include <vector>

#include "i2c.hpp"
#include "mag_connection.hpp"

class MagSlave : public MagConnection {
 public:
  bool Initialize(std::shared_ptr<I2C> i2c, const MagConfig& conf) override;
  bool WriteByte(const uint8_t reg, const uint8_t data) const override;
  std::vector<uint8_t> ReadBytes(const uint8_t reg,
                                 const uint8_t count) const override;

 protected:
  bool Prepare(const uint8_t reg, const bool is_read) const;
  bool EnableTransmit(const uint8_t count) const;
  bool SetData(const uint8_t data) const;
};

#endif  // RC_MAG_SLAVE_HPP
