// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_EVENT_LEVEL_HPP_
#define CORE_UTILS_EVENT_LEVEL_HPP_

#include <iostream>
#include <string>

namespace core::utils {
/**
 * @brief Defines the level of an event to be deal with
 *
 */
enum class EventLevel : uint8_t {
  DEBUG = 1 << 0,  // only for information used in debug mode
  INFO = 1 << 1,   // basic level of information needed to be reported
  WARN = 1 << 2,   // object behaviour is not desirable but not fatal
  ERROR = 1 << 3,  // object has error and should treat it
  FATAL = 1 << 4,  // object has fatal critical  error and system should abort
  CRITICAL = WARN | ERROR | FATAL,
  ALL = DEBUG | INFO | WARN | ERROR | FATAL,
};

/**
 * @brief Returns the name of the EventLevel
 *
 * @param event event level
 * @return const std::string& name of the event
 */
const std::string& EventLevelToString(const EventLevel event);

/**
 * @brief Stream EventLevel name
 *
 * @param os output stream
 * @param event event level
 * @return std::ostream& stream reference with EventLevel name appended
 */
std::ostream& operator<<(std::ostream& os, const EventLevel event);

// inline bool IsEventLevel(const EventLevel::type event1,
//                          const EventLevel::type event2) {
//   return event1 & event2;
// }

// inline bool IsCritical(const EventLevel::type event) {
//   return IsEventLevel(EventLevel::CRITICAL, event);
// }
}  // namespace core::utils
#endif  // CORE_UTILS_EVENT_LEVEL_HPP_
