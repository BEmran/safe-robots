// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_COMMON_LITERALS_HPP_
#define HARDWARE_COMMON_LITERALS_HPP_

#include <cstdint>

namespace hardware::common {
/**
 * @brief User litterer to define a number of type uint8_t
 *
 */
inline constexpr uint8_t operator"" _uc(
  unsigned long long arg) noexcept {  // NOLINT [runtime/int] TODO(Bara)
  return static_cast<uint8_t>(arg);
}
// using namespace literals;  // NOLINT [build/namespaces_literals] TODO(Bara)

}  // namespace hardware::common
#endif  // HARDWARE_COMMON_LITERALS_HPP_
