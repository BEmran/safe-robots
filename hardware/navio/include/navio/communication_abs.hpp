#ifndef _MPU_COMMUNICATION_ABS_HPP
#define _MPU_COMMUNICATION_ABS_HPP

#include <stdint.h>

#include <vector>

namespace navio
{
class CommunicationAbs
{
 public:
  CommunicationAbs(const bool debug) : debug_(debug){};

  virtual ~CommunicationAbs()
  {
  }

  virtual void WriteRegister(const uint8_t reg, const uint8_t data) const = 0;

  virtual void WriteRegisters(
      const std::vector<std::pair<uint8_t, uint8_t>>& reg_and_data) const = 0;

  virtual uint8_t ReadRegister(const uint8_t reg) const = 0;

  virtual std::vector<uint8_t> ReadRegisters(const uint8_t reg,
                                             const uint8_t count) const = 0;

  inline bool IsDebug() const
  {
    return debug_;
  }

 private:
  bool debug_;
};
}  // namespace navio
#endif  //_MPU_COMMUNICATION_ABS_HPP