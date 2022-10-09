// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_EVENT_LEVEL_HPP_
#define CORE_UTILS_EVENT_LEVEL_HPP_

#include <memory>
#include <string>
#include <string_view>

#include "core/utils/modifier.hpp"

namespace core::utils {
/**
 * @brief Defines the level of an event to be deal with
 *
 */
enum class EventLevel {
  INFO = 0 << 1,   // basic level of information
  DEBUG = 1 << 1,  // only for information used in debug mode
  WARN = 2 << 1,   // when system behaviour is not desirable but not fatal
  ERROR = 3 << 1,  // when system is fatal and system should treat it
};
// struct EventLevel {
//   enum type : uint8_t {
//     INFO = 0 << 1,   // basic level of information
//     DEBUG = 1 << 1,  // only for information used in debug mode
//     WARN = 2 << 1,   // when system behaviour is not desirable but not fatal
//     ERROR = 3 << 1,  // when system is fatal and system should treat it
//     CRITICAL = WARN | ERROR,
//     TRACE = DEBUG | CRITICAL,
//     ALL = INFO | TRACE,
//   };
// };

// inline bool IsEventLevel(const EventLevel::type event1,
//                          const EventLevel::type event2) {
//   return event1 & event2;
// }

// inline bool IsTrace(const EventLevel::type event) {
//   return IsEventLevel(EventLevel::TRACE, event);
// }

// inline bool IsCritical(const EventLevel::type event) {
//   return IsEventLevel(EventLevel::CRITICAL, event);
// }

/**
 * @brief Returns the name of the EventLevel
 *
 * @param event event level
 * @return std::string name of the event
 */
std::string EventLevelToString(EventLevel event);

}  // namespace core::utils
#endif  // CORE_UTILS_EVENT_LEVEL_HPP_
